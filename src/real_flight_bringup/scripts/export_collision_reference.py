#!/usr/bin/env python3

import csv
import os

import rospy
from quadrotor_msgs.msg import CollisionTrajectory


def _load_traj_csv(path: str):
    if not path or not os.path.exists(path):
        return None
    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                rows.append(
                    {
                        "time": float(row["time"]),
                        "x": float(row["px"]),
                        "y": float(row["py"]),
                        "z": float(row["pz"]),
                        "vx": float(row["vx"]),
                        "vy": float(row["vy"]),
                        "vz": float(row["vz"]),
                        "ax": float(row["ax"]),
                        "ay": float(row["ay"]),
                        "az": float(row["az"]),
                    }
                )
            except (KeyError, ValueError):
                continue
    return rows if len(rows) >= 2 else None


def _vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _lerp(a, b, r):
    return a + (b - a) * r


def _sample_traj_state(rows, t):
    if not rows:
        return None
    if t <= rows[0]["time"]:
        return dict(rows[0])
    if t >= rows[-1]["time"]:
        return dict(rows[-1])

    for idx in range(1, len(rows)):
        prev_row = rows[idx - 1]
        next_row = rows[idx]
        t0 = prev_row["time"]
        t1 = next_row["time"]
        if t <= t1:
            if t1 - t0 < 1e-9:
                return dict(next_row)
            ratio = (t - t0) / (t1 - t0)
            sampled = {"time": t}
            for key in ["x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az"]:
                sampled[key] = _lerp(prev_row[key], next_row[key], ratio)
            return sampled
    return dict(rows[-1])


class CollisionReferenceExporter:
    def __init__(self):
        self.collision_topic = rospy.get_param("~collision_topic", "/complete_collision_trajectory")
        self.output_csv = rospy.get_param(
            "~output_csv",
            "/home/liet/Catkin_Ego/Aerobatic-Planner-main/real_coll/data_/collision_reference.csv",
        )
        self.stage1_csv = rospy.get_param(
            "~stage1_csv",
            "/home/liet/Catkin_Ego/Aerobatic-Planner-main/real_coll/data_/final_traj.csv",
        )
        self.stage2_csv = rospy.get_param(
            "~stage2_csv",
            "/home/liet/Catkin_Ego/Aerobatic-Planner-main/real_coll/data_/optimized_traj.csv",
        )
        self.collision_transition_duration = max(
            1e-3, float(rospy.get_param("~collision_transition_duration", 0.15))
        )
        self.sample_dt = max(1e-3, float(rospy.get_param("~sample_dt", 0.01)))
        self.auto_shutdown_after_save = bool(rospy.get_param("~auto_shutdown_after_save", True))

        self.latest_collision = None
        self.saved = False
        self.last_stage1_mtime = None
        self.last_stage2_mtime = None

        rospy.Subscriber(self.collision_topic, CollisionTrajectory, self._collision_cb, queue_size=1)
        rospy.Timer(rospy.Duration.from_sec(0.5), self._timer_cb)

    def _collision_cb(self, msg: CollisionTrajectory):
        if not msg.collision_events:
            return
        self.latest_collision = msg
        self.saved = False
        rospy.loginfo(
            "export_collision_reference: received collision event at t=%.3f, waiting for stage CSVs to stitch reference",
            float(msg.collision_events[0].collision_time),
        )

    def _files_ready(self):
        if self.latest_collision is None:
            return False
        return os.path.exists(self.stage1_csv) and os.path.exists(self.stage2_csv)

    def _csvs_updated(self):
        changed = False
        for attr, path in [("last_stage1_mtime", self.stage1_csv), ("last_stage2_mtime", self.stage2_csv)]:
            if not os.path.exists(path):
                continue
            mtime = os.path.getmtime(path)
            if getattr(self, attr) != mtime:
                setattr(self, attr, mtime)
                changed = True
        return changed

    def _build_reference(self):
        stage1 = _load_traj_csv(self.stage1_csv)
        stage2 = _load_traj_csv(self.stage2_csv)
        if stage1 is None or stage2 is None or self.latest_collision is None:
            return None

        event = self.latest_collision.collision_events[0]
        tc = float(event.collision_time)
        event_pc = [float(event.collision_point.x), float(event.collision_point.y), float(event.collision_point.z)]
        stage1_at_collision = _sample_traj_state(stage1, tc)
        if stage1_at_collision is not None:
            pc = [
                float(stage1_at_collision["x"]),
                float(stage1_at_collision["y"]),
                float(stage1_at_collision["z"]),
            ]
        else:
            pc = event_pc
        pc_error = _vec_sub(pc, event_pc)
        pc_error_norm = (pc_error[0] ** 2 + pc_error[1] ** 2 + pc_error[2] ** 2) ** 0.5
        if pc_error_norm > 1e-3:
            rospy.logwarn(
                "export_collision_reference: stage1 position at collision time differs from event collision point by %.4fm, "
                "use stage1 position for continuous offline reference",
                pc_error_norm,
            )

        stitched = []

        for row in stage1:
            if row["time"] < tc - 1e-9:
                stitched.append(
                    {
                        "t_sec": row["time"],
                        "x": row["x"],
                        "y": row["y"],
                        "z": row["z"],
                        "vx": row["vx"],
                        "vy": row["vy"],
                        "vz": row["vz"],
                        "ax": row["ax"],
                        "ay": row["ay"],
                        "az": row["az"],
                        "jx": 0.0,
                        "jy": 0.0,
                        "jz": 0.0,
                        "yaw": 0.0,
                        "yaw_dot": 0.0,
                        "trajectory_id": 1,
                        "trajectory_flag": 1,
                    }
                )
            else:
                break

        hold_yaw = stitched[-1]["yaw"] if stitched else 0.0
        stitched.append(
            {
                "t_sec": tc,
                "x": pc[0],
                "y": pc[1],
                "z": pc[2],
                "vx": 0.0,
                "vy": 0.0,
                "vz": 0.0,
                "ax": 0.0,
                "ay": 0.0,
                "az": 0.0,
                "jx": 0.0,
                "jy": 0.0,
                "jz": 0.0,
                "yaw": hold_yaw,
                "yaw_dot": 0.0,
                "trajectory_id": 1,
                "trajectory_flag": 1,
            }
        )

        tau = self.sample_dt
        while tau <= self.collision_transition_duration + 1e-9:
            stitched.append(
                {
                    "t_sec": tc + tau,
                    "x": pc[0],
                    "y": pc[1],
                    "z": pc[2],
                    "vx": 0.0,
                    "vy": 0.0,
                    "vz": 0.0,
                    "ax": 0.0,
                    "ay": 0.0,
                    "az": 0.0,
                    "jx": 0.0,
                    "jy": 0.0,
                    "jz": 0.0,
                    "yaw": hold_yaw,
                    "yaw_dot": 0.0,
                    "trajectory_id": 1,
                    "trajectory_flag": 1,
                }
            )
            tau += self.sample_dt

        p_blend_end = [pc[0], pc[1], pc[2]]
        p2 = [stage2[0]["x"], stage2[0]["y"], stage2[0]["z"]]
        dp = _vec_sub(p_blend_end, p2)

        t_offset = tc + self.collision_transition_duration
        t0_stage2 = stage2[0]["time"]
        for idx, row in enumerate(stage2):
            if idx == 0:
                continue
            stitched.append(
                {
                    "t_sec": t_offset + (row["time"] - t0_stage2),
                    "x": row["x"] + dp[0],
                    "y": row["y"] + dp[1],
                    "z": row["z"] + dp[2],
                    "vx": row["vx"],
                    "vy": row["vy"],
                    "vz": row["vz"],
                    "ax": row["ax"],
                    "ay": row["ay"],
                    "az": row["az"],
                    "jx": 0.0,
                    "jy": 0.0,
                    "jz": 0.0,
                    "yaw": 0.0,
                    "yaw_dot": 0.0,
                    "trajectory_id": 2,
                    "trajectory_flag": 1,
                }
            )

        return stitched

    def _save_csv(self, rows):
        if not rows:
            return
        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)
        with open(self.output_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "t_sec",
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
                    "trajectory_id",
                    "trajectory_flag",
                ]
            )
            for row in rows:
                writer.writerow(
                    [
                        f"{row['t_sec']:.6f}",
                        f"{row['x']:.9f}",
                        f"{row['y']:.9f}",
                        f"{row['z']:.9f}",
                        f"{row['vx']:.9f}",
                        f"{row['vy']:.9f}",
                        f"{row['vz']:.9f}",
                        f"{row['ax']:.9f}",
                        f"{row['ay']:.9f}",
                        f"{row['az']:.9f}",
                        f"{row['jx']:.9f}",
                        f"{row['jy']:.9f}",
                        f"{row['jz']:.9f}",
                        f"{row['yaw']:.9f}",
                        f"{row['yaw_dot']:.9f}",
                        row["trajectory_id"],
                        row["trajectory_flag"],
                    ]
                )

    def _timer_cb(self, _unused_event):
        _ = _unused_event
        if self.saved or not self._files_ready():
            return
        self._csvs_updated()
        rows = self._build_reference()
        if rows is None:
            return
        self._save_csv(rows)
        self.saved = True
        rospy.loginfo(
            "export_collision_reference: stitched %d samples from stage1+blend+stage2 to %s",
            len(rows),
            self.output_csv,
        )
        if self.auto_shutdown_after_save:
            rospy.signal_shutdown("collision reference exported")


def main():
    rospy.init_node("export_collision_reference", anonymous=False)
    CollisionReferenceExporter()
    rospy.spin()


if __name__ == "__main__":
    main()
