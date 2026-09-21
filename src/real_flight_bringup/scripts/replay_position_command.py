#!/usr/bin/env python3

import csv
import math
import os

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


def _load_reference_csv(path: str):
    if not path or not os.path.exists(path):
        raise RuntimeError(f"reference csv not found: {path}")

    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                {
                    "t_sec": float(row["t_sec"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "z": float(row["z"]),
                    "vx": float(row["vx"]),
                    "vy": float(row["vy"]),
                    "vz": float(row["vz"]),
                    "ax": float(row["ax"]),
                    "ay": float(row["ay"]),
                    "az": float(row["az"]),
                    "jx": float(row.get("jx", 0.0)),
                    "jy": float(row.get("jy", 0.0)),
                    "jz": float(row.get("jz", 0.0)),
                    "yaw": float(row.get("yaw", 0.0)),
                    "yaw_dot": float(row.get("yaw_dot", 0.0)),
                    "trajectory_id": int(row.get("trajectory_id", 1)),
                    "trajectory_flag": int(row.get("trajectory_flag", PositionCommand.TRAJECTORY_STATUS_READY)),
                }
            )

    if len(rows) < 2:
        raise RuntimeError(f"reference csv has insufficient rows: {path}")

    rows.sort(key=lambda item: item["t_sec"])
    return rows


def _lerp(a: float, b: float, r: float) -> float:
    return a + (b - a) * r


def _interpolate_rows(rows, t_sec: float):
    if t_sec <= rows[0]["t_sec"]:
        return dict(rows[0])
    if t_sec >= rows[-1]["t_sec"]:
        return dict(rows[-1])

    for idx in range(1, len(rows)):
        prev_row = rows[idx - 1]
        next_row = rows[idx]
        t0 = prev_row["t_sec"]
        t1 = next_row["t_sec"]
        if t_sec <= t1:
            if t1 - t0 < 1e-9:
                return dict(next_row)
            ratio = (t_sec - t0) / (t1 - t0)
            out = {
                "t_sec": t_sec,
                "trajectory_id": prev_row["trajectory_id"],
                "trajectory_flag": prev_row["trajectory_flag"],
            }
            for key in [
                "x",
                "y",
                "z",
                "vx",
                "vy",
                "vz",
                "ax",
                "ay",
                "az",
                "jx",
                "jy",
                "jz",
                "yaw",
                "yaw_dot",
            ]:
                out[key] = _lerp(prev_row[key], next_row[key], ratio)
            return out

    return dict(rows[-1])


class ReplayPositionCommand:
    def __init__(self):
        self.reference_csv = rospy.get_param(
            "~reference_csv",
            "/home/liet/Catkin_Ego/Aerobatic-Planner-main/real_coll/data_/collision_reference.csv",
        )
        self.cmd_topic = rospy.get_param("~cmd_topic", "/position_command")
        self.odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.rate_hz = max(1.0, float(rospy.get_param("~rate_hz", 50.0)))
        self.wait_start_trigger = bool(rospy.get_param("~wait_start_trigger", True))
        self.start_trigger_topic = rospy.get_param("~start_trigger_topic", "/traj_start_trigger")
        self.start_trigger_timeout_sec = float(rospy.get_param("~start_trigger_timeout_sec", 0.0))
        self.wait_offboard = bool(rospy.get_param("~wait_offboard", True))
        self.offboard_wait_timeout_sec = float(rospy.get_param("~offboard_wait_timeout_sec", 0.0))
        self.start_delay_sec = max(0.0, float(rospy.get_param("~start_delay_sec", 0.0)))
        self.hold_final_point = bool(rospy.get_param("~hold_final_point", True))
        self.start_warn_threshold = max(0.0, float(rospy.get_param("~start_warn_threshold", 0.5)))

        self.rows = _load_reference_csv(self.reference_csv)
        self.first_row = self.rows[0]
        self.last_row = self.rows[-1]
        self.first_t = float(self.first_row["t_sec"])
        self.last_t = float(self.last_row["t_sec"])

        self.state = None
        self.odom = None
        self.start_trigger_received = False
        self.start_trigger_pose = None
        self.replay_started = False
        self.replay_start_time = None

        self.cmd_pub = rospy.Publisher(self.cmd_topic, PositionCommand, queue_size=20)
        self.state_sub = rospy.Subscriber(self.state_topic, State, self._state_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        self.start_trigger_sub = rospy.Subscriber(
            self.start_trigger_topic, PoseStamped, self._start_trigger_cb, queue_size=1
        )

        rospy.loginfo(
            "replay_position_command: loaded %d rows from %s, t=[%.3f, %.3f]",
            len(self.rows),
            self.reference_csv,
            self.first_t,
            self.last_t,
        )

    def _state_cb(self, msg: State):
        self.state = msg

    def _odom_cb(self, msg: Odometry):
        self.odom = msg

    def _start_trigger_cb(self, msg: PoseStamped):
        self.start_trigger_pose = msg
        if not self.start_trigger_received:
            self.start_trigger_received = True
            rospy.loginfo("replay_position_command: received start trigger on %s", self.start_trigger_topic)

    def _offboard_ready(self) -> bool:
        return self.state is not None and self.state.mode.upper() == "OFFBOARD"

    def _publish_row(self, row):
        msg = PositionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.position.x = row["x"]
        msg.position.y = row["y"]
        msg.position.z = row["z"]
        msg.velocity.x = row["vx"]
        msg.velocity.y = row["vy"]
        msg.velocity.z = row["vz"]
        msg.acceleration.x = row["ax"]
        msg.acceleration.y = row["ay"]
        msg.acceleration.z = row["az"]
        msg.jerk.x = row["jx"]
        msg.jerk.y = row["jy"]
        msg.jerk.z = row["jz"]
        msg.yaw = row["yaw"]
        msg.yaw_dot = row["yaw_dot"]
        msg.trajectory_id = int(row["trajectory_id"])
        msg.trajectory_flag = int(row["trajectory_flag"])
        self.cmd_pub.publish(msg)

    def _warn_start_offset_once(self):
        if self.odom is None:
            rospy.logwarn_throttle(2.0, "replay_position_command: waiting for odom, cannot evaluate start offset yet")
            return

        dx = float(self.odom.pose.pose.position.x) - self.first_row["x"]
        dy = float(self.odom.pose.pose.position.y) - self.first_row["y"]
        dz = float(self.odom.pose.pose.position.z) - self.first_row["z"]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist > self.start_warn_threshold:
            rospy.logwarn_throttle(
                2.0,
                "replay_position_command: current odom is %.3fm away from reference start [dx=%.3f dy=%.3f dz=%.3f]",
                dist,
                dx,
                dy,
                dz,
            )

    def _wait_for_start_condition(self):
        if self.wait_start_trigger:
            start = rospy.Time.now()
            rate = rospy.Rate(max(1.0, min(20.0, self.rate_hz)))
            while not rospy.is_shutdown():
                if self.start_trigger_received:
                    return True
                if self.start_trigger_timeout_sec > 0.0:
                    if (rospy.Time.now() - start).to_sec() >= self.start_trigger_timeout_sec:
                        return False
                rate.sleep()
            return False

        if not self.wait_offboard:
            return True

        start = rospy.Time.now()
        rate = rospy.Rate(max(1.0, min(20.0, self.rate_hz)))
        while not rospy.is_shutdown():
            self._publish_row(self.first_row)
            self._warn_start_offset_once()
            if self._offboard_ready():
                return True
            if self.offboard_wait_timeout_sec > 0.0:
                if (rospy.Time.now() - start).to_sec() >= self.offboard_wait_timeout_sec:
                    return False
            rate.sleep()
        return False

    def run(self):
        rospy.sleep(0.2)
        if not self._wait_for_start_condition():
            if self.wait_start_trigger:
                rospy.logerr("replay_position_command: start trigger wait timeout, abort replay")
            else:
                rospy.logerr("replay_position_command: OFFBOARD wait timeout, abort replay")
            return

        self._warn_start_offset_once()

        if self.start_delay_sec > 0.0:
            if self.wait_start_trigger:
                rospy.loginfo(
                    "replay_position_command: start trigger detected, hold first point for %.3fs",
                    self.start_delay_sec,
                )
            else:
                rospy.loginfo("replay_position_command: OFFBOARD detected, hold first point for %.3fs", self.start_delay_sec)
            end_time = rospy.Time.now() + rospy.Duration.from_sec(self.start_delay_sec)
            rate = rospy.Rate(self.rate_hz)
            while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                self._publish_row(self.first_row)
                rate.sleep()

        self.replay_started = True
        self.replay_start_time = rospy.Time.now()
        rospy.loginfo("replay_position_command: start replay on %s", self.cmd_topic)

        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - self.replay_start_time).to_sec()
            sample_time = self.first_t + max(0.0, elapsed)

            if sample_time <= self.last_t:
                row = _interpolate_rows(self.rows, sample_time)
                self._publish_row(row)
            else:
                if self.hold_final_point:
                    self._publish_row(self.last_row)
                else:
                    rospy.loginfo("replay_position_command: replay finished")
                    return

            rate.sleep()


def main():
    rospy.init_node("replay_position_command", anonymous=False)
    ReplayPositionCommand().run()


if __name__ == "__main__":
    main()
