#!/usr/bin/env python3
"""Interactively record collision/avoidance flight waypoints from ROS pose."""

import math
import os
import sys
import tempfile
import threading

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


MODE_TO_ID = {"avoidance": 1, "collision": 2}
MODE_ALIASES = {
    "avoid": "avoidance",
    "avoidance": "avoidance",
    "避碰": "avoidance",
    "collision": "collision",
    "collide": "collision",
    "碰撞": "collision",
}


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    norm = math.sqrt(
        quaternion.x * quaternion.x + quaternion.y * quaternion.y +
        quaternion.z * quaternion.z + quaternion.w * quaternion.w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid zero/non-finite quaternion")
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def canonical_mode(value):
    mode = MODE_ALIASES.get(str(value).strip().lower())
    if mode is None:
        raise ValueError("mode must be avoidance/避碰 or collision/碰撞")
    return mode


class FlightWaypointRecorder:
    def __init__(self):
        self._lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._latest_pose = None
        self._latest_stamp = rospy.Time()
        self._latest_source_frame = ""

        self._pose_topic = rospy.get_param(
            "~pose_topic", "/mavros/local_position/pose")
        self._pose_type = str(rospy.get_param("~pose_type", "pose")).lower()
        self._output_file = os.path.abspath(os.path.expanduser(rospy.get_param(
            "~output_file", "/Users/yyx/Documents/catkin_ws/src/real_flight_bringup/config/flight_waypoints.yaml")))
        self._frame_id = str(rospy.get_param("~frame_id", "world"))
        self._mode = canonical_mode(rospy.get_param(
            "~default_flight_mode", "avoidance"))
        self._max_pose_age = float(rospy.get_param("~max_pose_age", 1.0))
        self._load_existing = bool(rospy.get_param("~load_existing", True))
        self._interactive = bool(rospy.get_param("~interactive", True))
        self._command_topic = str(rospy.get_param(
            "~command_topic", "/waypoint_recorder/command"))
        self._waypoints = []

        if self._pose_type not in ("pose", "odometry", "odom"):
            raise ValueError("~pose_type must be pose or odometry")
        if not math.isfinite(self._max_pose_age) or self._max_pose_age <= 0.0:
            raise ValueError("~max_pose_age must be finite and positive")

        if self._load_existing and os.path.isfile(self._output_file):
            self._load_file()

        if self._pose_type == "pose":
            self._subscriber = rospy.Subscriber(
                self._pose_topic, PoseStamped, self._pose_callback,
                queue_size=10)
        else:
            self._subscriber = rospy.Subscriber(
                self._pose_topic, Odometry, self._odometry_callback,
                queue_size=10)
        self._command_subscriber = rospy.Subscriber(
            self._command_topic, String, self._command_callback, queue_size=10)

    def _cache_pose(self, pose, header):
        stamp = header.stamp if not header.stamp.is_zero() else rospy.Time.now()
        with self._lock:
            self._latest_pose = pose
            self._latest_stamp = stamp
            self._latest_source_frame = header.frame_id

    def _pose_callback(self, message):
        self._cache_pose(message.pose, message.header)

    def _odometry_callback(self, message):
        self._cache_pose(message.pose.pose, message.header)

    def _snapshot_pose(self):
        with self._lock:
            return (self._latest_pose, self._latest_stamp,
                    self._latest_source_frame)

    def _validate_waypoint(self, waypoint, expected_id):
        if not isinstance(waypoint, dict):
            raise ValueError("waypoint must be a mapping")
        mode = canonical_mode(waypoint.get("flight_mode", ""))
        point_id = int(waypoint.get("id", expected_id))
        if point_id != expected_id:
            raise ValueError("waypoint ids must be contiguous and start at 0")
        trajectory_mode = int(waypoint.get(
            "trajectory_mode", MODE_TO_ID[mode]))
        if trajectory_mode != MODE_TO_ID[mode]:
            raise ValueError(
                "waypoint trajectory_mode conflicts with flight_mode")
        values = {}
        for field in ("x", "y", "z", "yaw"):
            values[field] = float(waypoint[field])
            if not math.isfinite(values[field]):
                raise ValueError("waypoint {} must be finite".format(field))
        return {
            "id": point_id,
            "flight_mode": mode,
            "trajectory_mode": trajectory_mode,
            "x": values["x"],
            "y": values["y"],
            "z": values["z"],
            "yaw": normalize_angle(values["yaw"]),
        }

    def _load_file(self):
        with open(self._output_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict):
            raise ValueError("existing waypoint file root must be a mapping")
        if data.get("schema_version") != 1:
            raise ValueError("existing waypoint schema_version must be 1")
        points = data.get("waypoints", [])
        if not isinstance(points, list):
            raise ValueError("existing waypoints must be a sequence")
        self._waypoints = [
            self._validate_waypoint(point, index)
            for index, point in enumerate(points)
        ]
        rospy.loginfo("Loaded %d existing waypoints from %s",
                      len(self._waypoints), self._output_file)

    def _document(self):
        return {
            "schema_version": 1,
            "frame_id": self._frame_id,
            "pose_topic": self._pose_topic,
            "mode_ids": {
                "avoidance": MODE_TO_ID["avoidance"],
                "collision": MODE_TO_ID["collision"],
            },
            "waypoints": self._waypoints,
        }

    def _write_file(self):
        directory = os.path.dirname(self._output_file)
        os.makedirs(directory, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=os.path.basename(self._output_file) + ".",
            suffix=".tmp", dir=directory, text=True)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                stream.write("# Flight waypoint file: yaw unit is rad.\n")
                stream.write("# trajectory_mode: 1=avoidance, 2=collision.\n")
                yaml.safe_dump(
                    self._document(), stream, allow_unicode=True,
                    sort_keys=False, default_flow_style=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self._output_file)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def _add_waypoint(self):
        pose, stamp, source_frame = self._snapshot_pose()
        if pose is None:
            rospy.logwarn("No pose received from %s", self._pose_topic)
            return
        age = (rospy.Time.now() - stamp).to_sec()
        if age < -0.1 or age > self._max_pose_age:
            rospy.logwarn("Pose is stale (age %.3f s); waypoint not recorded", age)
            return
        p = pose.position
        values = (p.x, p.y, p.z)
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn("Pose XYZ contains a non-finite value")
            return
        try:
            yaw = normalize_angle(yaw_from_quaternion(pose.orientation))
        except ValueError as error:
            rospy.logwarn("Cannot record waypoint: %s", error)
            return

        waypoint = {
            "id": len(self._waypoints),
            "flight_mode": self._mode,
            "trajectory_mode": MODE_TO_ID[self._mode],
            "x": round(float(p.x), 6),
            "y": round(float(p.y), 6),
            "z": round(float(p.z), 6),
            "yaw": round(float(yaw), 6),
        }
        self._waypoints.append(waypoint)
        self._write_file()
        rospy.loginfo(
            "Recorded %d [%s]: x=%.3f y=%.3f z=%.3f yaw=%.3f rad (frame=%s)",
            waypoint["id"], self._mode, waypoint["x"], waypoint["y"],
            waypoint["z"], waypoint["yaw"], source_frame or "<empty>")

    def _undo(self):
        if not self._waypoints:
            rospy.logwarn("No waypoint to undo")
            return
        point = self._waypoints.pop()
        self._write_file()
        rospy.loginfo("Removed waypoint id=%d", point["id"])

    def _handle_command(self, command):
        with self._command_lock:
            lowered = command.strip().lower()
            if lowered in ("", "add", "a"):
                self._add_waypoint()
            elif lowered == "1":
                self._mode = "avoidance"
                rospy.loginfo("Mode: avoidance (避碰)")
            elif lowered == "2":
                self._mode = "collision"
                rospy.loginfo("Mode: collision (计划碰撞)")
            elif lowered in ("undo", "u"):
                self._undo()
            elif lowered in ("save", "s"):
                self._write_file()
                rospy.loginfo("Saved %d waypoints to %s",
                              len(self._waypoints), self._output_file)
            elif lowered in ("quit", "q", "exit"):
                self._write_file()
                return False
            else:
                rospy.logwarn("Unknown command: %s", command)
            return True

    def _command_callback(self, message):
        rospy.loginfo("Waypoint command topic: %s", message.data)
        if not self._handle_command(message.data):
            rospy.signal_shutdown("quit command received")

    def run(self):
        rospy.loginfo("Waypoint recorder: %s [%s] -> %s",
                      self._pose_topic, self._pose_type, self._output_file)
        print("Commands: Enter/a record | 1 avoidance | 2 collision | u undo | s save | q save & quit")
        print("Current mode: {}. Yaw is read from the current pose.".format(self._mode))

        if not self._interactive:
            rospy.loginfo("Non-interactive mode; send std_msgs/String commands to %s",
                          self._command_topic)
            rospy.spin()
            self._write_file()
            return

        while not rospy.is_shutdown():
            try:
                command = input("waypoint> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not self._handle_command(command):
                break

        self._write_file()
        rospy.loginfo("Saved %d waypoints to %s and exited",
                      len(self._waypoints), self._output_file)


def main():
    rospy.init_node("record_flight_waypoints")
    try:
        FlightWaypointRecorder().run()
    except Exception as error:
        rospy.logerr("Waypoint recorder failed: %s", error)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
