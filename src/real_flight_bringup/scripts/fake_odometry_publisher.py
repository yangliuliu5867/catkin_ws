#!/usr/bin/env python3

import math

import rospy
from nav_msgs.msg import Odometry


def yaw_to_quaternion(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class FakeOdometryPublisher:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.rate_hz = max(1.0, float(rospy.get_param("~rate_hz", 30.0)))
        self.start_param = rospy.get_param("~start_param", "/PlanningStart")
        self.start_yaw = float(rospy.get_param("~start_yaw", 0.0))

        start = rospy.get_param(self.start_param, [0.0, 0.0, 1.0])
        if not isinstance(start, (list, tuple)) or len(start) < 3:
            raise RuntimeError(f"Invalid start parameter at {self.start_param}")
        self.start_x = float(start[0])
        self.start_y = float(start[1])
        self.start_z = float(start[2])

        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=1, latch=True)

    def run(self):
        qx, qy, qz, qw = yaw_to_quaternion(self.start_yaw)
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.child_frame_id = self.child_frame_id
            msg.pose.pose.position.x = self.start_x
            msg.pose.pose.position.y = self.start_y
            msg.pose.pose.position.z = self.start_z
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = 0.0
            self.pub.publish(msg)
            rate.sleep()


def main():
    rospy.init_node("fake_odometry_publisher", anonymous=False)
    FakeOdometryPublisher().run()


if __name__ == "__main__":
    main()
