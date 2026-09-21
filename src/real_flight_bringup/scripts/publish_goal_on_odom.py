#!/usr/bin/env python3

import rospy
import rosgraph
import re
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class GoalPublisherOnOdom:
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
            # Accept forms like "[a, b]" or "a, b" or "a b".
            s = re.sub(r"^[\[\(\{]\s*|\s*[\]\)\}]$", "", s)
            parts = [p.strip() for p in re.split(r"[\s,]+", s) if p.strip()]
            return parts
        return [str(value)]

    def __init__(self):
        self.topic = rospy.get_param("~topic", "/goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.setpos_param = rospy.get_param("~setpos_param", "/SetPos")
        self.delay_sec = float(rospy.get_param("~delay_sec", 0.0))
        self.repeat_count = int(rospy.get_param("~repeat_count", 5))
        self.repeat_hz = float(rospy.get_param("~repeat_hz", 10.0))
        self.odom_wait_timeout_sec = float(rospy.get_param("~odom_wait_timeout_sec", 20.0))
        self.wait_for_map_topic = rospy.get_param("~wait_for_map_topic", "")
        self.map_wait_timeout_sec = float(rospy.get_param("~map_wait_timeout_sec", 30.0))
        self.wait_for_subscriber_nodes = self._coerce_str_list(rospy.get_param("~wait_for_subscriber_nodes", []))
        self.subscriber_wait_timeout_sec = float(rospy.get_param("~subscriber_wait_timeout_sec", 20.0))

        self.has_odom = False
        self.pub = rospy.Publisher(self.topic, PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)

    def _odom_cb(self, _msg: Odometry):
        self.has_odom = True

    def _get_goal(self):
        if not rospy.has_param(self.setpos_param):
            raise RuntimeError(f"Parameter not found: {self.setpos_param}")
        values = rospy.get_param(self.setpos_param)
        if not isinstance(values, (list, tuple)) or len(values) < 3:
            raise RuntimeError(f"Invalid SetPos in {self.setpos_param}")
        return float(values[-3]), float(values[-2]), float(values[-1])

    @staticmethod
    def _norm_node(name: str) -> str:
        if not name:
            return ""
        return name if name.startswith("/") else "/" + name

    def _wait_for_named_subscribers(self):
        if not self.wait_for_subscriber_nodes:
            return

        want = {self._norm_node(n) for n in self.wait_for_subscriber_nodes if str(n).strip()}
        if not want:
            return

        master = rosgraph.Master(rospy.get_name())
        start = rospy.Time.now()
        last_report = rospy.Time(0)

        while not rospy.is_shutdown():
            if self.subscriber_wait_timeout_sec > 0.0 and (rospy.Time.now() - start).to_sec() > self.subscriber_wait_timeout_sec:
                rospy.logerr(
                    "publish_goal_on_odom: timeout waiting for subscribers %s on %s",
                    sorted(want),
                    self.topic,
                )
                return False

            try:
                pubs, subs, _srvs = master.getSystemState()
            except Exception as exc:
                rospy.logwarn_throttle(1.0, "publish_goal_on_odom: failed to query master system state: %s", exc)
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
                rospy.loginfo("publish_goal_on_odom: waiting for %s to subscribe to %s (have=%s)", sorted(missing), self.topic, sorted(sub_nodes))

            rospy.sleep(0.1)

        return False

    def run(self):
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.has_odom:
                break
            if self.odom_wait_timeout_sec > 0.0 and (rospy.Time.now() - start).to_sec() > self.odom_wait_timeout_sec:
                rospy.logerr("publish_goal_on_odom: odom wait timeout")
                return
            rospy.sleep(0.05)

        if self.delay_sec > 0.0:
            rospy.sleep(self.delay_sec)

        if self.wait_for_map_topic:
            rospy.loginfo("publish_goal_on_odom: waiting for map message on %s", self.wait_for_map_topic)
            try:
                rospy.wait_for_message(self.wait_for_map_topic, PointCloud2, timeout=self.map_wait_timeout_sec if self.map_wait_timeout_sec > 0.0 else None)
            except Exception as exc:
                rospy.logerr("publish_goal_on_odom: map wait timeout/failure on %s: %s", self.wait_for_map_topic, exc)
                return

        if not self._wait_for_named_subscribers():
            return

        try:
            gx, gy, gz = self._get_goal()
        except Exception as exc:
            rospy.logerr(f"publish_goal_on_odom failed to read goal: {exc}")
            return

        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = gz
        msg.pose.orientation.w = 1.0

        rate = rospy.Rate(max(1.0, self.repeat_hz))
        for _ in range(max(1, self.repeat_count)):
            if rospy.is_shutdown():
                return
            msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)
            rate.sleep()

        rospy.loginfo("publish_goal_on_odom: published goal to %s [%.3f, %.3f, %.3f]", self.topic, gx, gy, gz)


def main():
    rospy.init_node("publish_goal_on_odom", anonymous=False)
    GoalPublisherOnOdom().run()


if __name__ == "__main__":
    main()
