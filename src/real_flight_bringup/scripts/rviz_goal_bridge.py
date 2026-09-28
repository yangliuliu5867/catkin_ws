#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped


class RvizGoalBridge:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/move_base_simple/goal")
        self.output_topic = rospy.get_param("~output_topic", "/goal")
        self.goal_z = float(rospy.get_param("~goal_z", 1.0))
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.publisher = rospy.Publisher(self.output_topic, PoseStamped, queue_size=1)
        self.subscriber = rospy.Subscriber(
            self.input_topic, PoseStamped, self.goal_callback, queue_size=1
        )

    def goal_callback(self, msg: PoseStamped):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id
        goal.pose = msg.pose
        goal.pose.position.z = self.goal_z
        if abs(goal.pose.orientation.x) + abs(goal.pose.orientation.y) + \
                abs(goal.pose.orientation.z) + abs(goal.pose.orientation.w) < 1.0e-6:
            goal.pose.orientation.w = 1.0
        self.publisher.publish(goal)
        rospy.loginfo(
            "RViz目标已转发: x=%.3f y=%.3f z=%.3f -> %s",
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z,
            self.output_topic,
        )


if __name__ == "__main__":
    rospy.init_node("rviz_goal_bridge")
    RvizGoalBridge()
    rospy.spin()
