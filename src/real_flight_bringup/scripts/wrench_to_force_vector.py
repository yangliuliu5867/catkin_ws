#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped, Vector3Stamped


class WrenchToForceVector:
    def __init__(self):
        in_topic = rospy.get_param("~input_wrench_topic", "/external_wrench_estimation/ewe_out")
        out_topic = rospy.get_param("~output_force_topic", "/external_force_est")
        frame_id = rospy.get_param("~output_frame_id", "world")

        self._frame_id = frame_id
        self._pub = rospy.Publisher(out_topic, Vector3Stamped, queue_size=20)
        self._sub = rospy.Subscriber(in_topic, WrenchStamped, self._cb, queue_size=20)

        rospy.loginfo(f"wrench_to_force_vector: {in_topic} -> {out_topic}")

    def _cb(self, msg: WrenchStamped):
        out = Vector3Stamped()
        out.header.stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        out.header.frame_id = msg.header.frame_id if msg.header.frame_id else self._frame_id
        out.vector.x = msg.wrench.force.x
        out.vector.y = msg.wrench.force.y
        out.vector.z = msg.wrench.force.z
        self._pub.publish(out)


def main():
    rospy.init_node("wrench_to_force_vector", anonymous=False)
    WrenchToForceVector()
    rospy.spin()


if __name__ == "__main__":
    main()
