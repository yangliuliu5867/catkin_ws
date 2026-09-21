#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


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


class DirectGcopterFrontend:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/position_command")
        self.rate_hz = max(1.0, float(rospy.get_param("~rate_hz", 50.0)))
        self.goal_z_floor = float(rospy.get_param("~goal_z_floor", 0.5))
        self.max_vel = max(1e-3, float(rospy.get_param("~max_vel", 0.8)))
        self.max_acc = max(1e-3, float(rospy.get_param("~max_acc", 1.2)))
        self.min_duration = max(0.2, float(rospy.get_param("~min_duration", 2.0)))
        self.hold_at_goal = bool(rospy.get_param("~hold_at_goal", True))

        self.latest_odom = None
        self.goal_active = False
        self.start_time = None
        self.duration = 0.0
        self.start_pos = (0.0, 0.0, 0.0)
        self.goal_pos = (0.0, 0.0, 0.0)
        self.hold_yaw = 0.0

        self.cmd_pub = rospy.Publisher(self.cmd_topic, PositionCommand, queue_size=20)

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_cb, queue_size=1)
        rospy.Timer(rospy.Duration.from_sec(1.0 / self.rate_hz), self._timer_cb)

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _goal_cb(self, msg: PoseStamped):
        if self.latest_odom is None:
            rospy.logwarn("direct_gcopter_frontend: no odom yet, ignore goal")
            return

        p0 = self.latest_odom.pose.pose.position
        q0 = self.latest_odom.pose.pose.orientation
        goal_z = max(float(msg.pose.position.z), self.goal_z_floor)

        self.start_pos = (float(p0.x), float(p0.y), float(p0.z))
        self.goal_pos = (float(msg.pose.position.x), float(msg.pose.position.y), goal_z)
        self.hold_yaw = _yaw_from_quaternion(q0)

        dx = self.goal_pos[0] - self.start_pos[0]
        dy = self.goal_pos[1] - self.start_pos[1]
        dz = self.goal_pos[2] - self.start_pos[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 1e-3:
            self.duration = self.min_duration
        else:
            t_v = (1.875 * dist) / self.max_vel
            t_a = math.sqrt((5.766 * dist) / self.max_acc)
            self.duration = max(self.min_duration, t_v, t_a)

        self.start_time = rospy.Time.now()
        self.goal_active = True

        rospy.loginfo(
            "direct_gcopter_frontend: start min-jerk to goal [%.3f, %.3f, %.3f], T=%.3f on %s",
            self.goal_pos[0], self.goal_pos[1], self.goal_pos[2], self.duration, self.cmd_pub.resolved_name
        )

    def _publish_cmd(self, stamp: rospy.Time, pos, vel, acc, jerk):
        cmd = PositionCommand()
        cmd.header.stamp = stamp
        cmd.header.frame_id = "world"
        cmd.position.x = pos[0]
        cmd.position.y = pos[1]
        cmd.position.z = pos[2]
        cmd.velocity.x = vel[0]
        cmd.velocity.y = vel[1]
        cmd.velocity.z = vel[2]
        cmd.acceleration.x = acc[0]
        cmd.acceleration.y = acc[1]
        cmd.acceleration.z = acc[2]
        cmd.jerk.x = jerk[0]
        cmd.jerk.y = jerk[1]
        cmd.jerk.z = jerk[2]
        cmd.yaw = self.hold_yaw
        cmd.yaw_dot = 0.0
        self.cmd_pub.publish(cmd)

    def _timer_cb(self, _unused_event):
        _ = _unused_event
        if not self.goal_active or self.start_time is None:
            return

        now = rospy.Time.now()
        elapsed = max(0.0, (now - self.start_time).to_sec())
        T = max(1e-6, self.duration)
        t = min(elapsed, T)
        s = max(0.0, min(1.0, t / T))

        phi = _min_jerk_quintic(s)
        dphi = _min_jerk_quintic_d1(s)
        ddphi = _min_jerk_quintic_d2(s)
        dddphi = _min_jerk_quintic_d3(s)

        delta = (
            self.goal_pos[0] - self.start_pos[0],
            self.goal_pos[1] - self.start_pos[1],
            self.goal_pos[2] - self.start_pos[2],
        )

        pos = (
            self.start_pos[0] + delta[0] * phi,
            self.start_pos[1] + delta[1] * phi,
            self.start_pos[2] + delta[2] * phi,
        )
        vel = (
            delta[0] * dphi / T,
            delta[1] * dphi / T,
            delta[2] * dphi / T,
        )
        acc = (
            delta[0] * ddphi / (T * T),
            delta[1] * ddphi / (T * T),
            delta[2] * ddphi / (T * T),
        )
        jerk = (
            delta[0] * dddphi / (T * T * T),
            delta[1] * dddphi / (T * T * T),
            delta[2] * dddphi / (T * T * T),
        )

        if elapsed >= T:
            pos = self.goal_pos
            vel = (0.0, 0.0, 0.0)
            acc = (0.0, 0.0, 0.0)
            jerk = (0.0, 0.0, 0.0)
            if not self.hold_at_goal:
                self.goal_active = False

        self._publish_cmd(now, pos, vel, acc, jerk)


def main():
    rospy.init_node("direct_gcopter_frontend", anonymous=False)
    DirectGcopterFrontend()
    rospy.spin()


if __name__ == "__main__":
    main()
