#!/usr/bin/env python3
"""Execute Mission.waypoints from mission.yaml with per-point mode and yaw."""

import math
import os
import threading

import rospy
import yaml
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from quadrotor_msgs.msg import FlightWaypoint, PolynomialTrajectory


VALID_MODES = {"avoidance": FlightWaypoint.MODE_COLLISION_FREE,
               "collision": FlightWaypoint.MODE_COLLISION}


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_yaw(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid odometry quaternion")
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def load_waypoints(path):
    with open(path, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not isinstance(document.get("Mission"), dict):
        raise ValueError("mission file must contain a Mission mapping")
    mission = document["Mission"]
    mission_mode = str(mission.get("mode", "")).strip().lower()
    if mission_mode not in ("single", "multi"):
        raise ValueError("Mission.mode must be single or multi")
    if mission_mode == "single":
        return None, []
    frame_id = str(mission.get("frame_id", "world"))
    raw_points = mission.get("waypoints")
    if not isinstance(raw_points, list) or not raw_points:
        raise ValueError("Mission.mode is multi but Mission.waypoints is empty")

    points = []
    for index, raw in enumerate(raw_points):
        if not isinstance(raw, dict) or int(raw.get("id", index)) != index:
            raise ValueError("waypoint ids must be contiguous and start at 0")
        mode_name = str(raw.get("flight_mode", "")).strip().lower()
        if mode_name not in VALID_MODES:
            raise ValueError("waypoint {} has invalid flight_mode".format(index))
        mode = int(raw.get("trajectory_mode", VALID_MODES[mode_name]))
        if mode != VALID_MODES[mode_name]:
            raise ValueError("waypoint {} mode fields conflict".format(index))
        point = {"id": index, "mode_name": mode_name, "mode": mode}
        for field in ("x", "y", "z", "yaw"):
            point[field] = float(raw[field])
            if not math.isfinite(point[field]):
                raise ValueError("waypoint {} {} is not finite".format(index, field))
        point["yaw"] = normalize_angle(point["yaw"])
        points.append(point)
    return frame_id, points


class FlightWaypointExecutor:
    def __init__(self):
        self._lock = threading.Lock()
        mission_file = os.path.abspath(os.path.expanduser(
            rospy.get_param("~mission_file", "/Users/yyx/Documents/catkin_ws/src/real_flight_bringup/config/mission.yaml")))
        self._frame_id, self._points = load_waypoints(mission_file)
        if not self._points:
            raise RuntimeError("Mission.mode is single; multi-point executor is inactive")
        self._goal_topic = rospy.get_param("~goal_topic", "/flight_waypoint_goal")
        self._odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom")
        self._remaining_topic = rospy.get_param(
            "~remaining_time_topic", "/trajectory_remaining_time")
        self._trajectory_topic = rospy.get_param(
            "~trajectory_topic", "/trajectory_generator_node/trajectory")
        self._command_topic = rospy.get_param(
            "~command_topic", "/flight_waypoint_executor/command")
        self._position_tolerance = float(rospy.get_param("~position_tolerance", 0.20))
        self._yaw_tolerance = float(rospy.get_param("~yaw_tolerance", 0.20))
        self._remaining_tolerance = float(rospy.get_param("~remaining_time_tolerance", 0.25))
        self._hold_time = float(rospy.get_param("~hold_time", 0.5))
        self._minimum_goal_age = float(rospy.get_param("~minimum_goal_age", 0.5))
        self._index = int(rospy.get_param("~start_index", 0))
        if self._index < 0 or self._index >= len(self._points):
            raise ValueError("~start_index is outside the waypoint list")
        for name, value in (("position_tolerance", self._position_tolerance),
                            ("yaw_tolerance", self._yaw_tolerance),
                            ("remaining_time_tolerance", self._remaining_tolerance),
                            ("hold_time", self._hold_time),
                            ("minimum_goal_age", self._minimum_goal_age)):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("~{} must be finite and non-negative".format(name))

        self._state = "IDLE"
        self._odom = None
        self._remaining = None
        self._remaining_stamp = rospy.Time()
        self._goal_stamp = rospy.Time()
        self._within_since = None
        self._trajectory_received = False

        self._goal_pub = rospy.Publisher(
            self._goal_topic, FlightWaypoint, queue_size=1, latch=True)
        self._status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True)
        self._odom_sub = rospy.Subscriber(
            self._odom_topic, Odometry, self._odom_callback, queue_size=10)
        self._remaining_sub = rospy.Subscriber(
            self._remaining_topic, Float64, self._remaining_callback, queue_size=10)
        self._trajectory_sub = rospy.Subscriber(
            self._trajectory_topic, PolynomialTrajectory,
            self._trajectory_callback, queue_size=10)
        self._command_sub = rospy.Subscriber(
            self._command_topic, String, self._command_callback, queue_size=10)
        self._timer = rospy.Timer(rospy.Duration(0.1), self._timer_callback)
        self._publish_status("loaded {} waypoint(s) from {}".format(
            len(self._points), mission_file))
        if bool(rospy.get_param("~autostart", False)):
            self._start()

    def _publish_status(self, detail=""):
        text = "state={} index={}/{}".format(
            self._state, min(self._index + 1, len(self._points)), len(self._points))
        if detail:
            text += " " + detail
        self._status_pub.publish(String(data=text))
        rospy.loginfo("Waypoint executor: %s", text)

    def _odom_callback(self, message):
        with self._lock:
            self._odom = message

    def _remaining_callback(self, message):
        if not math.isfinite(message.data):
            return
        with self._lock:
            self._remaining = max(0.0, message.data)
            self._remaining_stamp = rospy.Time.now()

    def _trajectory_callback(self, message):
        with self._lock:
            stamp = message.header.stamp
            if (not self._goal_stamp.is_zero() and
                    (stamp.is_zero() or stamp >= self._goal_stamp)):
                self._trajectory_received = True

    def _publish_current_goal(self):
        point = self._points[self._index]
        message = FlightWaypoint()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self._frame_id
        message.id = point["id"]
        message.total_waypoints = len(self._points)
        message.position.x = point["x"]
        message.position.y = point["y"]
        message.position.z = point["z"]
        message.flight_mode = point["mode"]
        message.yaw = point["yaw"]
        self._goal_stamp = message.header.stamp
        self._remaining = None
        self._within_since = None
        self._trajectory_received = False
        self._goal_pub.publish(message)
        self._publish_status(
            "published mode={} xyz=({:.3f},{:.3f},{:.3f}) yaw={:.3f}".format(
                point["mode_name"], point["x"], point["y"], point["z"], point["yaw"]))

    def _start(self):
        with self._lock:
            if self._state == "COMPLETE":
                self._index = 0
            if self._odom is None:
                self._state = "WAITING_ODOM"
                self._publish_status("waiting for odometry before publishing")
            else:
                self._state = "RUNNING"
                self._publish_current_goal()

    def _command_callback(self, message):
        command = message.data.strip().lower()
        if command in ("start", "resume"):
            self._start()
        elif command == "pause":
            with self._lock:
                self._state = "PAUSED"
                self._within_since = None
                self._publish_status("automatic advancement paused")
        elif command == "stop":
            with self._lock:
                self._state = "IDLE"
                self._within_since = None
                self._publish_status("sequencing stopped")
        elif command == "retry":
            with self._lock:
                self._state = "RUNNING"
                self._publish_current_goal()
        elif command == "next":
            with self._lock:
                self._advance(force=True)
        elif command == "status":
            with self._lock:
                self._publish_status()
        else:
            rospy.logwarn("Unknown waypoint command '%s'; use start/pause/resume/stop/retry/next/status", command)

    def _advance(self, force=False):
        if not force and self._state != "RUNNING":
            return
        if self._index + 1 >= len(self._points):
            self._state = "COMPLETE"
            self._within_since = None
            self._publish_status("mission complete")
            return
        self._index += 1
        self._state = "RUNNING"
        self._publish_current_goal()

    def _timer_callback(self, _event):
        with self._lock:
            if self._state == "WAITING_ODOM" and self._odom is not None:
                self._state = "RUNNING"
                self._publish_current_goal()
                return
            if self._state != "RUNNING" or self._odom is None or self._remaining is None:
                return
            if not self._trajectory_received:
                return
            now = rospy.Time.now()
            if ((now - self._goal_stamp).to_sec() < self._minimum_goal_age or
                    self._remaining_stamp < self._goal_stamp):
                return
            point = self._points[self._index]
            pose = self._odom.pose.pose
            dx = pose.position.x - point["x"]
            dy = pose.position.y - point["y"]
            dz = pose.position.z - point["z"]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            try:
                yaw_error = abs(normalize_angle(quaternion_yaw(pose.orientation) - point["yaw"]))
            except ValueError:
                self._within_since = None
                return
            arrived = (distance <= self._position_tolerance and
                       yaw_error <= self._yaw_tolerance and
                       self._remaining <= self._remaining_tolerance)
            if not arrived:
                self._within_since = None
                return
            if self._within_since is None:
                self._within_since = now
                return
            if (now - self._within_since).to_sec() >= self._hold_time:
                rospy.loginfo("Waypoint %d reached: distance=%.3f yaw_error=%.3f remaining=%.3f",
                              self._index, distance, yaw_error, self._remaining)
                self._advance()


def main():
    rospy.init_node("execute_flight_waypoints")
    try:
        FlightWaypointExecutor()
    except RuntimeError as error:
        rospy.loginfo("%s", error)
        return 0
    except (OSError, ValueError, yaml.YAMLError) as error:
        rospy.logfatal("Cannot start waypoint executor: %s", error)
        return 1
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
