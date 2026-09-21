#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PolynomialTrajectory
from quadrotor_msgs.msg import PositionCommand


def _attitude_error_to_identity(q) -> float:
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + (q.w - 1.0) * (q.w - 1.0))


def _tilt_angle_from_quaternion(q) -> float:
    """Return tilt angle (rad) between body z-axis and world z-axis (yaw-invariant)."""
    r33 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    r33 = max(-1.0, min(1.0, r33))
    return math.acos(r33)


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


class _Cache:
    def __init__(self, state_topic: str, odom_topic: str):
        self.state = None
        self.odom = None
        self._state_sub = rospy.Subscriber(state_topic, State, self._state_cb, queue_size=1)
        self._odom_sub = rospy.Subscriber(odom_topic, Odometry, self._odom_cb, queue_size=1)

    def _state_cb(self, msg: State):
        self.state = msg

    def _odom_cb(self, msg: Odometry):
        self.odom = msg


class ControllerStabilityTest:
    def __init__(self):
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom")
        self.pre_align_cmd_topic = rospy.get_param("~pre_align_cmd_topic", "/position_command")
        self.start_trigger_topic = rospy.get_param("~start_trigger_topic", "/traj_start_trigger")

        self.wait_offboard = bool(rospy.get_param("~wait_offboard", True))
        self.offboard_wait_timeout_sec = float(rospy.get_param("~offboard_wait_timeout_sec", 0.0))
        self.enable_pre_align = bool(rospy.get_param("~enable_pre_align", True))
        self.pre_align_altitude = float(rospy.get_param("~pre_align_altitude", 1.5))
        self.pre_align_rate_hz = float(rospy.get_param("~pre_align_rate_hz", 30.0))
        self.pre_align_hold_sec = float(rospy.get_param("~pre_align_hold_sec", 0.8))
        self.pre_align_timeout_sec = float(rospy.get_param("~pre_align_timeout_sec", 20.0))
        self.pre_align_z_tolerance = float(rospy.get_param("~pre_align_z_tolerance", 0.15))
        self.pre_align_att_tolerance = float(rospy.get_param("~pre_align_att_tolerance", 0.20))
        self.pre_align_blind_publish_sec = float(rospy.get_param("~pre_align_blind_publish_sec", 3.0))

        self.pre_align_max_vel = float(rospy.get_param("~pre_align_max_vel", 0.6))
        self.pre_align_max_acc = float(rospy.get_param("~pre_align_max_acc", 1.2))
        self.pre_align_min_duration = float(rospy.get_param("~pre_align_min_duration", 2.0))

        self.start_publish_delay_sec = float(rospy.get_param("~start_publish_delay_sec", 0.5))
        self.start_publish_count = int(rospy.get_param("~start_publish_count", 20))
        self.start_publish_hz = float(rospy.get_param("~start_publish_hz", 20.0))
        self.frame_id = rospy.get_param("~frame_id", "world")

        self.wait_traj_before_start_trigger = bool(rospy.get_param("~wait_traj_before_start_trigger", False))
        self.traj_topic = rospy.get_param("~traj_topic", "/trajectory_generator_node/trajectory")
        self.traj_wait_timeout_sec = float(rospy.get_param("~traj_wait_timeout_sec", 0.0))
        self._has_traj = False

        self.cache = _Cache(self.state_topic, self.odom_topic)
        self.pre_align_cmd_pub = rospy.Publisher(self.pre_align_cmd_topic, PositionCommand, queue_size=20)
        self.start_trigger_pub = rospy.Publisher(self.start_trigger_topic, PoseStamped, queue_size=20)

        if self.wait_traj_before_start_trigger:
            self._traj_sub = rospy.Subscriber(self.traj_topic, PolynomialTrajectory, self._traj_cb, queue_size=1)
        else:
            self._traj_sub = None

    def _traj_cb(self, msg: PolynomialTrajectory):
        # Mark trajectory ready when we see any valid trajectory message.
        if msg is None:
            return
        if getattr(msg, "num_segment", 0) <= 0:
            return
        self._has_traj = True

    def _wait_for_traj(self) -> bool:
        if not self.wait_traj_before_start_trigger:
            return True
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._has_traj:
                return True
            if self.traj_wait_timeout_sec > 0.0 and (rospy.Time.now() - start).to_sec() > self.traj_wait_timeout_sec:
                return False
            rospy.sleep(0.05)
        return False

    def _wait_for_odom(self, timeout_sec: float) -> bool:
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.cache.odom is not None:
                return True
            if timeout_sec > 0.0 and (rospy.Time.now() - start).to_sec() > timeout_sec:
                return False
            rospy.sleep(0.05)
        return False

    def _wait_for_offboard(self) -> bool:
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            st = self.cache.state
            if st is not None and st.mode.upper() == "OFFBOARD":
                return True
            if self.offboard_wait_timeout_sec > 0.0 and (rospy.Time.now() - start).to_sec() > self.offboard_wait_timeout_sec:
                return False
            rospy.sleep(0.05)
        return False

    def _run_pre_align(self) -> bool:
        odom0 = self.cache.odom
        if odom0 is None:
            rospy.logerr("controller_stability_test: no odom for pre-align")
            return False

        x0 = float(odom0.pose.pose.position.x)
        y0 = float(odom0.pose.pose.position.y)
        z0 = float(odom0.pose.pose.position.z)
        yaw0 = _yaw_from_quaternion(odom0.pose.pose.orientation)

        dz = float(self.pre_align_altitude) - z0
        abs_dz = abs(dz)
        if abs_dz < 1e-3:
            return True

        max_vel = max(1e-3, float(self.pre_align_max_vel))
        max_acc = max(1e-3, float(self.pre_align_max_acc))
        min_duration = max(0.2, float(self.pre_align_min_duration))

        t_v = (1.875 * abs_dz) / max_vel
        t_a = math.sqrt((5.766 * abs_dz) / max_acc)
        T = max(min_duration, t_v, t_a)

        start = rospy.Time.now()
        reached_since = None
        rate = rospy.Rate(max(1.0, self.pre_align_rate_hz))

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
            cmd.header.frame_id = self.frame_id
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
            self.pre_align_cmd_pub.publish(cmd)

            odom = self.cache.odom
            if odom is not None:
                cur = odom.pose.pose
                z_ok = abs(cur.position.z - self.pre_align_altitude) <= self.pre_align_z_tolerance
                att_ok = _tilt_angle_from_quaternion(cur.orientation) <= self.pre_align_att_tolerance
                if z_ok and att_ok:
                    if reached_since is None:
                        reached_since = now
                    elif (now - reached_since).to_sec() >= self.pre_align_hold_sec:
                        return True
                else:
                    reached_since = None
            elif elapsed >= self.pre_align_blind_publish_sec:
                return True

            if self.pre_align_timeout_sec > 0.0 and elapsed >= self.pre_align_timeout_sec:
                return False

            rate.sleep()

        return False

    def _publish_start_trigger(self):
        odom = self.cache.odom
        if odom is None:
            rospy.logerr("controller_stability_test: no odom for start trigger")
            return

        msg = PoseStamped()
        msg.header.frame_id = "start"
        msg.pose = odom.pose.pose
        msg.pose.position.z = self.pre_align_altitude if self.enable_pre_align else odom.pose.pose.position.z

        rate = rospy.Rate(max(1.0, self.start_publish_hz))
        for _ in range(max(1, self.start_publish_count)):
            if rospy.is_shutdown():
                return
            msg.header.stamp = rospy.Time.now()
            self.start_trigger_pub.publish(msg)
            rate.sleep()

    def run(self):
        if not self._wait_for_odom(timeout_sec=10.0):
            rospy.logerr("controller_stability_test: no odom, abort")
            return

        if self.wait_offboard:
            rospy.loginfo("controller_stability_test: waiting OFFBOARD")
            if not self._wait_for_offboard():
                rospy.logerr("controller_stability_test: OFFBOARD wait timeout, abort")
                return

        if self.enable_pre_align:
            rospy.loginfo("controller_stability_test: pre-align start")
            if not self._run_pre_align():
                rospy.logerr("controller_stability_test: pre-align failed, abort")
                return
        else:
            rospy.loginfo("controller_stability_test: pre-align disabled")

        if self.start_publish_delay_sec > 0.0:
            rospy.sleep(self.start_publish_delay_sec)

        if self.wait_traj_before_start_trigger:
            rospy.loginfo(f"controller_stability_test: waiting for trajectory on {self.traj_topic}")
            if not self._wait_for_traj():
                rospy.logerr("controller_stability_test: trajectory wait timeout, abort")
                return

        rospy.loginfo("controller_stability_test: publish start trigger for collision controller test")
        self._publish_start_trigger()
        rospy.loginfo("controller_stability_test: start trigger sent, controller test running")
        rospy.spin()


def main():
    rospy.init_node("controller_stability_test", anonymous=False)
    ControllerStabilityTest().run()


if __name__ == "__main__":
    main()
