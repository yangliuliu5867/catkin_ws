#!/usr/bin/env python3

import threading

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import CollisionTrajectory


class FakeCollisionTriggerPublisher:
    def __init__(self):
        self.collision_topic = rospy.get_param("~collision_topic", "/complete_collision_trajectory")
        self.trigger_topic = rospy.get_param("~trigger_topic", "/collision_trigger")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.transition_duration = max(0.0, float(rospy.get_param("~transition_duration", 0.15)))
        self.hold_sec = max(0.05, float(rospy.get_param("~hold_sec", 0.15)))
        self.extra_delay_sec = max(0.0, float(rospy.get_param("~extra_delay_sec", 0.0)))

        self.pub = rospy.Publisher(self.trigger_topic, Bool, queue_size=1, latch=True)
        self.last_signature = None
        self.generation = 0
        self.lock = threading.Lock()

        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_cb, queue_size=1)
        rospy.Subscriber(self.collision_topic, CollisionTrajectory, self._collision_cb, queue_size=1)

        # Keep default low level false.
        self.pub.publish(Bool(data=False))

    def _goal_cb(self, _msg: PoseStamped):
        with self.lock:
            self.generation += 1
            self.last_signature = None
        self.pub.publish(Bool(data=False))

    def _collision_cb(self, msg: CollisionTrajectory):
        if not msg.collision_events:
            return

        event = msg.collision_events[0]
        signature = (
            round(float(event.collision_time), 3),
            round(float(event.collision_point.x), 3),
            round(float(event.collision_point.y), 3),
            round(float(event.collision_point.z), 3),
        )

        with self.lock:
            if signature == self.last_signature:
                return
            self.last_signature = signature
            generation = self.generation

        delay = max(0.0, float(event.collision_time) + self.transition_duration + self.extra_delay_sec)
        rospy.loginfo(
            "publish_fake_collision_trigger: schedule /collision_trigger after %.3fs for collision at t=%.3f",
            delay,
            float(event.collision_time),
        )
        timer = threading.Timer(delay, self._pulse_true, args=(generation,))
        timer.daemon = True
        timer.start()

    def _pulse_true(self, generation: int):
        with self.lock:
            if generation != self.generation:
                return
        self.pub.publish(Bool(data=True))
        timer = threading.Timer(self.hold_sec, self._pulse_false, args=(generation,))
        timer.daemon = True
        timer.start()

    def _pulse_false(self, generation: int):
        with self.lock:
            if generation != self.generation:
                return
        self.pub.publish(Bool(data=False))


def main():
    rospy.init_node("publish_fake_collision_trigger", anonymous=False)
    FakeCollisionTriggerPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
