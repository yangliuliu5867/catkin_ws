#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


class _StateCache:
    def __init__(self, state_topic: str, odom_topic: str):
        self.state = None
        self.odom = None
        self._state_sub = rospy.Subscriber(state_topic, State, self._state_cb, queue_size=1)
        self._odom_sub = rospy.Subscriber(odom_topic, Odometry, self._odom_cb, queue_size=1)

    def _state_cb(self, msg: State):
        self.state = msg

    def _odom_cb(self, msg: Odometry):
        self.odom = msg


def _wait_for_odom(cache: _StateCache, timeout_sec: float) -> bool:
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        if cache.odom is not None:
            return True
        if timeout_sec > 0.0 and (rospy.Time.now() - start).to_sec() > timeout_sec:
            return False
        rospy.sleep(0.05)
    return False


def _get_goal_from_setpos(param_name: str):
    if not rospy.has_param(param_name):
        raise RuntimeError(f"Parameter not found: {param_name}")

    values = rospy.get_param(param_name)
    if not isinstance(values, (list, tuple)) or len(values) < 3 or (len(values) % 3) != 0:
        raise RuntimeError(f"Invalid SetPos format in {param_name}: expected flat [x1,y1,z1,...]")

    # Use final waypoint as mission endpoint.
    return float(values[-3]), float(values[-2]), float(values[-1])


def _wait_for_offboard(cache: _StateCache, timeout_sec: float) -> bool:
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        st = cache.state
        if st is not None and st.mode.upper() == "OFFBOARD":
            rospy.loginfo("publish_goal_once: OFFBOARD detected, start pre-align stage")
            return True
        if timeout_sec > 0.0 and (rospy.Time.now() - start).to_sec() > timeout_sec:
            return False
        rospy.sleep(0.05)
    return False


def _attitude_error_to_identity(q) -> float:
    # Deprecated: this metric penalizes yaw as well and can block pre-align if
    # the vehicle starts with non-zero yaw.
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + (q.w - 1.0) * (q.w - 1.0))


def _tilt_angle_from_quaternion(q) -> float:
    """Return tilt angle (rad) between body z-axis and world z-axis.

    This is yaw-invariant and therefore suitable for judging whether the
    vehicle is level (roll/pitch small) regardless of heading.

    Assumes q describes the body frame orientation in the world frame
    (standard ROS Pose orientation semantics).
    """
    # Third column of rotation matrix (body z-axis expressed in world frame).
    # cos(tilt) = z_world · (R * e_z_body) = r33 = 1 - 2(x^2 + y^2)
    r33 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    r33 = max(-1.0, min(1.0, r33))
    return math.acos(r33)


def _yaw_from_quaternion(q) -> float:
    # ROS uses x,y,z,w
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _min_jerk_quintic(s: float) -> float:
    # 10 s^3 - 15 s^4 + 6 s^5, s in [0,1]
    return (10.0 * s ** 3) - (15.0 * s ** 4) + (6.0 * s ** 5)


def _min_jerk_quintic_d1(s: float) -> float:
    # d/ds of min-jerk quintic: 30 s^2 - 60 s^3 + 30 s^4
    return (30.0 * s ** 2) - (60.0 * s ** 3) + (30.0 * s ** 4)


def _min_jerk_quintic_d2(s: float) -> float:
    # d^2/ds^2: 60 s - 180 s^2 + 120 s^3
    return (60.0 * s) - (180.0 * s ** 2) + (120.0 * s ** 3)


def _min_jerk_quintic_d3(s: float) -> float:
    # d^3/ds^3: 60 - 360 s + 360 s^2
    return 60.0 - (360.0 * s) + (360.0 * s ** 2)


def _publish_pre_align_position_command(cache: _StateCache,
                                       pub: rospy.Publisher,
                                       frame_id: str,
                                       target_altitude: float,
                                       rate_hz: float,
                                       hold_sec: float,
                                       timeout_sec: float,
                                       z_tolerance: float,
                                       att_tolerance: float,
                                       blind_publish_sec: float,
                                       max_vel: float,
                                       max_acc: float,
                                       min_duration: float) -> bool:
    odom0 = cache.odom
    if odom0 is None:
        rospy.logerr("publish_goal_once: no odom for PositionCommand pre-align")
        return False

    x0 = float(odom0.pose.pose.position.x)
    y0 = float(odom0.pose.pose.position.y)
    z0 = float(odom0.pose.pose.position.z)
    yaw0 = _yaw_from_quaternion(odom0.pose.pose.orientation)

    dz = float(target_altitude) - z0
    abs_dz = abs(dz)
    if abs_dz < 1e-3:
        rospy.loginfo("publish_goal_once: already near target altitude, skip pre-align climb")
        return True

    max_vel = max(1e-3, float(max_vel))
    max_acc = max(1e-3, float(max_acc))
    min_duration = max(0.2, float(min_duration))

    # For min-jerk quintic:
    # v_max = 1.875 * |dz| / T
    # a_max = 5.766... * |dz| / T^2
    t_v = (1.875 * abs_dz) / max_vel
    t_a = math.sqrt((5.766 * abs_dz) / max_acc)
    T = max(min_duration, t_v, t_a)

    rospy.loginfo(
        "publish_goal_once: pre-align via PositionCommand to z=%.3f (dz=%.3f, T=%.3f, vmax<=%.3f, amax<=%.3f) on %s",
        target_altitude,
        dz,
        T,
        max_vel,
        max_acc,
        pub.resolved_name,
    )

    start = rospy.Time.now()
    reached_since = None
    rate = rospy.Rate(max(1.0, rate_hz))

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        elapsed = (now - start).to_sec()

        # Generate reference (time-based) to avoid stalling on noisy odom.
        t = max(0.0, min(elapsed, T))
        s = 0.0 if T <= 1e-6 else (t / T)
        s = max(0.0, min(1.0, s))

        z = z0 + dz * _min_jerk_quintic(s)
        vz = (dz / T) * _min_jerk_quintic_d1(s)
        az = (dz / (T * T)) * _min_jerk_quintic_d2(s)
        jz = (dz / (T * T * T)) * _min_jerk_quintic_d3(s)

        cmd = PositionCommand()
        cmd.header.stamp = now
        cmd.header.frame_id = frame_id
        cmd.position.x = x0
        cmd.position.y = y0
        cmd.position.z = z
        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = vz
        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = az
        cmd.jerk.x = 0.0
        cmd.jerk.y = 0.0
        cmd.jerk.z = jz
        cmd.yaw = yaw0
        cmd.yaw_dot = 0.0
        pub.publish(cmd)

        odom_now = cache.odom
        if odom_now is not None:
            cur_pose = odom_now.pose.pose
            z_ok = abs(cur_pose.position.z - target_altitude) <= z_tolerance
            att_ok = _tilt_angle_from_quaternion(cur_pose.orientation) <= att_tolerance
            if z_ok and att_ok:
                if reached_since is None:
                    reached_since = now
                elif (now - reached_since).to_sec() >= hold_sec:
                    return True
            else:
                reached_since = None
        elif elapsed >= blind_publish_sec:
            rospy.logwarn("publish_goal_once: pre-align finished by blind publish timeout (no odom feedback)")
            return True

        if timeout_sec > 0.0 and elapsed >= timeout_sec:
            rospy.logerr("publish_goal_once: pre-align timeout")
            return False

        rate.sleep()


def _run_pre_align(cache: _StateCache,
                   pub: rospy.Publisher,
                   fallback_frame_id: str,
                   target_altitude: float,
                   rate_hz: float,
                   hold_sec: float,
                   timeout_sec: float,
                   z_tolerance: float,
                   att_tolerance: float,
                   blind_publish_sec: float) -> bool:
    target = PoseStamped()
    target.pose.orientation.w = 1.0

    odom = cache.odom
    if odom is not None:
        target.header.frame_id = odom.header.frame_id if odom.header.frame_id else fallback_frame_id
        target.pose.position.x = odom.pose.pose.position.x
        target.pose.position.y = odom.pose.pose.position.y
    else:
        target.header.frame_id = fallback_frame_id
        target.pose.position.x = 0.0
        target.pose.position.y = 0.0
        rospy.logwarn("publish_goal_once: no odom yet for pre-align, fallback xy=(0,0)")

    target.pose.position.z = target_altitude
    rospy.loginfo(
        "publish_goal_once: pre-align target [x=%.3f, y=%.3f, z=%.3f, q=(0,0,0,1)] on %s",
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z,
        target.header.frame_id,
    )

    start = rospy.Time.now()
    reached_since = None
    rate = rospy.Rate(max(1.0, rate_hz))

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        elapsed = (now - start).to_sec()

        target.header.stamp = now
        pub.publish(target)

        odom_now = cache.odom
        if odom_now is not None:
            cur_pose = odom_now.pose.pose
            z_ok = abs(cur_pose.position.z - target_altitude) <= z_tolerance
            att_ok = _attitude_error_to_identity(cur_pose.orientation) <= att_tolerance
            if z_ok and att_ok:
                if reached_since is None:
                    reached_since = now
                elif (now - reached_since).to_sec() >= hold_sec:
                    return True
            else:
                reached_since = None
        elif elapsed >= blind_publish_sec:
            rospy.logwarn("publish_goal_once: pre-align finished by blind publish timeout (no odom feedback)")
            return True

        if timeout_sec > 0.0 and elapsed >= timeout_sec:
            rospy.logerr("publish_goal_once: pre-align timeout")
            return False

        rate.sleep()

    return False


def main():
    rospy.init_node("publish_goal_once", anonymous=False)

    setpos_param = rospy.get_param("~setpos_param", "/SetPos")
    topic = rospy.get_param("~topic", "/goal")
    frame_id = rospy.get_param("~frame_id", "world")
    delay_sec = float(rospy.get_param("~delay_sec", 3.0))
    repeat_count = int(rospy.get_param("~repeat_count", 5))
    repeat_hz = float(rospy.get_param("~repeat_hz", 10.0))

    enable_pre_align = bool(rospy.get_param("~enable_pre_align", False))
    wait_offboard = bool(rospy.get_param("~wait_offboard", True))
    offboard_wait_timeout_sec = float(rospy.get_param("~offboard_wait_timeout_sec", 0.0))
    state_topic = rospy.get_param("~state_topic", "/mavros/state")
    odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom")

    pre_align_cmd_topic = rospy.get_param("~pre_align_cmd_topic", "/position_command")
    pre_align_altitude = float(rospy.get_param("~pre_align_altitude", 1.5))
    pre_align_rate_hz = float(rospy.get_param("~pre_align_rate_hz", 30.0))
    pre_align_hold_sec = float(rospy.get_param("~pre_align_hold_sec", 0.8))
    pre_align_timeout_sec = float(rospy.get_param("~pre_align_timeout_sec", 20.0))
    pre_align_z_tolerance = float(rospy.get_param("~pre_align_z_tolerance", 0.15))
    pre_align_att_tolerance = float(rospy.get_param("~pre_align_att_tolerance", 0.20))
    pre_align_blind_publish_sec = float(rospy.get_param("~pre_align_blind_publish_sec", 3.0))
    odom_wait_timeout_sec = float(rospy.get_param("~odom_wait_timeout_sec", 10.0))

    pre_align_max_vel = float(rospy.get_param("~pre_align_max_vel", 0.6))
    pre_align_max_acc = float(rospy.get_param("~pre_align_max_acc", 1.2))
    pre_align_min_duration = float(rospy.get_param("~pre_align_min_duration", 2.0))

    pub = rospy.Publisher(topic, PoseStamped, queue_size=1)

    if enable_pre_align:
        cache = _StateCache(state_topic, odom_topic)
        pre_align_cmd_pub = rospy.Publisher(pre_align_cmd_topic, PositionCommand, queue_size=20)
        rospy.sleep(0.2)

        rospy.loginfo("publish_goal_once: waiting for odom before pre-align")
        if not _wait_for_odom(cache, odom_wait_timeout_sec):
            rospy.logerr("publish_goal_once: odom wait timeout, abort goal publish")
            return

        if wait_offboard:
            rospy.loginfo("publish_goal_once: waiting OFFBOARD mode before pre-align")
            if not _wait_for_offboard(cache, offboard_wait_timeout_sec):
                rospy.logerr("publish_goal_once: OFFBOARD wait timeout, abort goal publish")
                return

        if not _publish_pre_align_position_command(
            cache,
            pre_align_cmd_pub,
            frame_id,
            pre_align_altitude,
            pre_align_rate_hz,
            pre_align_hold_sec,
            pre_align_timeout_sec,
            pre_align_z_tolerance,
            pre_align_att_tolerance,
            pre_align_blind_publish_sec,
            pre_align_max_vel,
            pre_align_max_acc,
            pre_align_min_duration,
        ):
            rospy.logerr("publish_goal_once: pre-align failed, abort goal publish")
            return

        rospy.loginfo("publish_goal_once: pre-align done, now publish global goal")

    if delay_sec > 0.0:
        rospy.sleep(delay_sec)

    try:
        gx, gy, gz = _get_goal_from_setpos(setpos_param)
    except Exception as exc:
        rospy.logerr(f"publish_goal_once failed to read endpoint: {exc}")
        return

    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = gx
    msg.pose.position.y = gy
    msg.pose.position.z = gz
    msg.pose.orientation.w = 1.0

    rate = rospy.Rate(max(1.0, repeat_hz))
    for _ in range(max(1, repeat_count)):
        if rospy.is_shutdown():
            break
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo(f"Published goal endpoint to {topic}: [{gx:.3f}, {gy:.3f}, {gz:.3f}]")


if __name__ == "__main__":
    main()
