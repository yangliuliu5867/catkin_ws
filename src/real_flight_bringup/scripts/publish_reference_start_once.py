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


def _wait_for_offboard(cache: _StateCache, timeout_sec: float) -> bool:
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        st = cache.state
        if st is not None and st.mode.upper() == "OFFBOARD":
            rospy.loginfo("publish_reference_start_once: OFFBOARD detected, start pre-align stage")
            return True
        if timeout_sec > 0.0 and (rospy.Time.now() - start).to_sec() > timeout_sec:
            return False
        rospy.sleep(0.05)
    return False


def _attitude_error_to_identity(q) -> float:
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + (q.w - 1.0) * (q.w - 1.0))


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _min_jerk_quintic(s: float) -> float:
    return (10.0 * s ** 3) - (15.0 * s ** 4) + (6.0 * s ** 5)


def _min_jerk_quintic_d1(s: float) -> float:
    return (30.0 * s ** 2) - (60.0 * s ** 3) + (30.0 * s ** 4)


def _min_jerk_quintic_d2(s: float) -> float:
    return (60.0 * s) - (180.0 * s ** 2) + (120.0 * s ** 3)


def _min_jerk_quintic_d3(s: float) -> float:
    return 60.0 - (360.0 * s) + (360.0 * s ** 2)


def _publish_pre_align_position_command(
    cache: _StateCache,
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
    min_duration: float,
) -> bool:
    odom0 = cache.odom
    if odom0 is None:
        rospy.logerr("publish_reference_start_once: no odom for PositionCommand pre-align")
        return False

    x0 = float(odom0.pose.pose.position.x)
    y0 = float(odom0.pose.pose.position.y)
    z0 = float(odom0.pose.pose.position.z)
    yaw0 = _yaw_from_quaternion(odom0.pose.pose.orientation)

    dz = float(target_altitude) - z0
    abs_dz = abs(dz)
    if abs_dz < 1e-3:
        rospy.loginfo("publish_reference_start_once: already near target altitude, skip pre-align climb")
        return True

    max_vel = max(1e-3, float(max_vel))
    max_acc = max(1e-3, float(max_acc))
    min_duration = max(0.2, float(min_duration))

    t_v = (1.875 * abs_dz) / max_vel
    t_a = math.sqrt((5.766 * abs_dz) / max_acc)
    T = max(min_duration, t_v, t_a)

    rospy.loginfo(
        "publish_reference_start_once: pre-align via PositionCommand to z=%.3f (dz=%.3f, T=%.3f)",
        target_altitude,
        dz,
        T,
    )

    start = rospy.Time.now()
    reached_since = None
    rate = rospy.Rate(max(1.0, rate_hz))

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        elapsed = (now - start).to_sec()

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
            att_ok = _attitude_error_to_identity(cur_pose.orientation) <= att_tolerance
            if z_ok and att_ok:
                if reached_since is None:
                    reached_since = now
                elif (now - reached_since).to_sec() >= hold_sec:
                    return True
            else:
                reached_since = None
        elif elapsed >= blind_publish_sec:
            rospy.logwarn("publish_reference_start_once: pre-align finished by blind publish timeout")
            return True

        if timeout_sec > 0.0 and elapsed >= timeout_sec:
            rospy.logerr("publish_reference_start_once: pre-align timeout")
            return False

        rate.sleep()

    return False


def main():
    rospy.init_node("publish_reference_start_once", anonymous=False)

    topic = rospy.get_param("~topic", "/traj_start_trigger")
    frame_id = rospy.get_param("~frame_id", "world")
    delay_sec = float(rospy.get_param("~delay_sec", 0.0))
    repeat_count = int(rospy.get_param("~repeat_count", 10))
    repeat_hz = float(rospy.get_param("~repeat_hz", 20.0))

    enable_pre_align = bool(rospy.get_param("~enable_pre_align", True))
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

        rospy.loginfo("publish_reference_start_once: waiting for odom before pre-align")
        if not _wait_for_odom(cache, odom_wait_timeout_sec):
            rospy.logerr("publish_reference_start_once: odom wait timeout, abort start trigger")
            return

        if wait_offboard:
            rospy.loginfo("publish_reference_start_once: waiting OFFBOARD mode before pre-align")
            if not _wait_for_offboard(cache, offboard_wait_timeout_sec):
                rospy.logerr("publish_reference_start_once: OFFBOARD wait timeout, abort start trigger")
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
            rospy.logerr("publish_reference_start_once: pre-align failed, abort start trigger")
            return

        rospy.loginfo("publish_reference_start_once: pre-align done, now publish replay start trigger")
        odom_for_trigger = cache.odom
    else:
        cache = _StateCache(state_topic, odom_topic)
        rospy.sleep(0.2)
        odom_for_trigger = cache.odom

    if delay_sec > 0.0:
        rospy.sleep(delay_sec)

    msg = PoseStamped()
    msg.header.frame_id = "start"
    if odom_for_trigger is not None:
        msg.pose = odom_for_trigger.pose.pose
    else:
        msg.pose.orientation.w = 1.0

    rate = rospy.Rate(max(1.0, repeat_hz))
    for _ in range(max(1, repeat_count)):
        if rospy.is_shutdown():
            break
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("publish_reference_start_once: published replay start trigger to %s", topic)


if __name__ == "__main__":
    main()
