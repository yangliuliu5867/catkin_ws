#!/usr/bin/env python3

import rospy
import rosgraph
import re
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class StartTriggerPublisher:
    @staticmethod
    def _coerce_str_list(value):
        if value is None:
            return []
        if isinstance(value, (list, tuple)):
            return [str(v) for v in value]
        if isinstance(value, str):
            s = value.strip()
            if not s:
                return []
            s = re.sub(r"^[\[\(\{]\s*|\s*[\]\)\}]$", "", s)
            parts = [p.strip() for p in re.split(r"[\s,]+", s) if p.strip()]
            return parts
        return [str(value)]

    def __init__(self):
        self.topic = rospy.get_param("~topic", "/traj_start_trigger")
        self.odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom")
        self.delay_sec = float(rospy.get_param("~delay_sec", 3.5))
        self.repeat_count = int(rospy.get_param("~repeat_count", 20))
        self.repeat_hz = float(rospy.get_param("~repeat_hz", 10.0))
        self.odom_wait_timeout_sec = float(rospy.get_param("~odom_wait_timeout_sec", 5.0))
        self.wait_for_subscriber_nodes = self._coerce_str_list(rospy.get_param("~wait_for_subscriber_nodes", []))
        self.subscriber_wait_timeout_sec = float(rospy.get_param("~subscriber_wait_timeout_sec", 20.0))
        self.has_odom = False
        self.latest_odom = None

        self.pub = rospy.Publisher(self.topic, PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)

    @staticmethod
    def _norm_node(name: str) -> str:
        if not name:
            return ""
        return name if name.startswith("/") else "/" + name

    def _wait_for_named_subscribers(self):
        if not self.wait_for_subscriber_nodes:
            return True

        want = {self._norm_node(n) for n in self.wait_for_subscriber_nodes if str(n).strip()}
        if not want:
            return True

        master = rosgraph.Master(rospy.get_name())
        start = rospy.Time.now()
        last_report = rospy.Time(0)

        while not rospy.is_shutdown():
            if self.subscriber_wait_timeout_sec > 0.0 and (rospy.Time.now() - start).to_sec() > self.subscriber_wait_timeout_sec:
                rospy.logerr(
                    "publish_start_trigger: timeout waiting for subscribers %s on %s",
                    sorted(want),
                    self.topic,
                )
                return False

            try:
                _pubs, subs, _srvs = master.getSystemState()
            except Exception as exc:
                rospy.logwarn_throttle(1.0, "publish_start_trigger: failed to query master system state: %s", exc)
                rospy.sleep(0.1)
                continue

            sub_nodes = set()
            for t, nodes in subs:
                if t == self.topic:
                    sub_nodes = {self._norm_node(n) for n in nodes}
                    break

            missing = want - sub_nodes
            if not missing:
                return True

            now = rospy.Time.now()
            if (now - last_report).to_sec() > 1.0:
                last_report = now
                rospy.loginfo("publish_start_trigger: waiting for %s to subscribe to %s (have=%s)", sorted(missing), self.topic, sorted(sub_nodes))

            rospy.sleep(0.1)

        return False

    def _odom_cb(self, msg: Odometry):
        self.has_odom = True
        self.latest_odom = msg

    def run(self):
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.has_odom:
                break
            if self.odom_wait_timeout_sec > 0.0 and (rospy.Time.now() - start).to_sec() > self.odom_wait_timeout_sec:
                rospy.logerr("publish_start_trigger: odom wait timeout")
                return
            rospy.sleep(0.05)

        if self.delay_sec > 0.0:
            rospy.sleep(self.delay_sec)

        if not self._wait_for_named_subscribers():
            return

        msg = PoseStamped()
        msg.header.frame_id = "start"
        if self.latest_odom is not None:
            msg.pose = self.latest_odom.pose.pose
        else:
            msg.pose.orientation.w = 1.0

        rate = rospy.Rate(max(1.0, self.repeat_hz))
        for _ in range(max(1, self.repeat_count)):
            if rospy.is_shutdown():
                return
            msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)
            rate.sleep()

        rospy.loginfo("publish_start_trigger: published start trigger to %s", self.topic)


def main():
    rospy.init_node("publish_start_trigger", anonymous=False)
    StartTriggerPublisher().run()


if __name__ == "__main__":
    main()
