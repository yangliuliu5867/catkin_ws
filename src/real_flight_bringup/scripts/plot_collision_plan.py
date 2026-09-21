#!/usr/bin/env python3

import csv
import os

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import CollisionTrajectory

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_traj_csv(path: str):
    if not path or not os.path.exists(path):
        return None
    points = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                points.append(
                    (
                        float(row["time"]),
                        float(row["px"]),
                        float(row["py"]),
                        float(row["pz"]),
                    )
                )
            except (KeyError, ValueError):
                continue
    return points if len(points) >= 2 else None


class CollisionPlanPlotter:
    def __init__(self):
        self.collision_topic = rospy.get_param("~collision_topic", "/complete_collision_trajectory")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom")
        self.output_png = rospy.get_param(
            "~output_png",
            "/home/liet/Catkin_Ego/Aerobatic-Planner-main/real_coll/data_/collision_plan.png",
        )
        self.stage1_csv = rospy.get_param(
            "~stage1_csv",
            "/home/liet/Catkin_Ego/Aerobatic-Planner-main/real_coll/data_/final_traj.csv",
        )
        self.stage2_csv = rospy.get_param(
            "~stage2_csv",
            "/home/liet/Catkin_Ego/Aerobatic-Planner-main/real_coll/data_/optimized_traj.csv",
        )
        self.save_period_sec = max(0.2, float(rospy.get_param("~save_period_sec", 1.0)))
        self.overwrite = bool(rospy.get_param("~overwrite", True))

        self.latest_collision = None
        self.latest_goal = None
        self.latest_odom = None
        self.last_save_time = None
        self.saved_once = False
        self.last_stage1_mtime = None
        self.last_stage2_mtime = None

        rospy.Subscriber(self.collision_topic, CollisionTrajectory, self._collision_cb, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_cb, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        rospy.Timer(rospy.Duration.from_sec(0.5), self._timer_cb)

    def _collision_cb(self, msg: CollisionTrajectory):
        self.latest_collision = msg
        self.saved_once = False

    def _goal_cb(self, msg: PoseStamped):
        self.latest_goal = msg
        self.saved_once = False

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _traj_files_updated(self):
        changed = False
        for attr, path in [("last_stage1_mtime", self.stage1_csv), ("last_stage2_mtime", self.stage2_csv)]:
            if not path or not os.path.exists(path):
                continue
            mtime = os.path.getmtime(path)
            if getattr(self, attr) != mtime:
                setattr(self, attr, mtime)
                changed = True
        return changed

    def _render(self):
        stage1 = load_traj_csv(self.stage1_csv)
        stage2 = load_traj_csv(self.stage2_csv)
        if stage1 is None and stage2 is None:
            return False

        os.makedirs(os.path.dirname(self.output_png), exist_ok=True)

        fig, axes = plt.subplots(2, 2, figsize=(12, 9))
        ax_xy = axes[0][0]
        ax_xz = axes[0][1]
        ax_t = axes[1][0]
        ax_yz = axes[1][1]

        def plot_series(points, label, color):
            t = [p[0] for p in points]
            x = [p[1] for p in points]
            y = [p[2] for p in points]
            z = [p[3] for p in points]
            ax_xy.plot(x, y, label=label, linewidth=2.0, color=color)
            ax_xz.plot(x, z, label=label, linewidth=2.0, color=color)
            ax_t.plot(t, x, label=f"{label}: x", color=color, linestyle="-")
            ax_t.plot(t, y, label=f"{label}: y", color=color, linestyle="--")
            ax_t.plot(t, z, label=f"{label}: z", color=color, linestyle=":")
            ax_yz.plot(y, z, label=label, linewidth=2.0, color=color)

        if stage1 is not None:
            plot_series(stage1, "stage-1 final_traj", "tab:blue")
        if stage2 is not None:
            plot_series(stage2, "stage-2 optimized_traj", "tab:purple")

        if self.latest_odom is not None:
            p = self.latest_odom.pose.pose.position
            ax_xy.scatter([p.x], [p.y], marker="o", s=50, label="current odom")
            ax_xz.scatter([p.x], [p.z], marker="o", s=50, label="current odom")
            ax_yz.scatter([p.y], [p.z], marker="o", s=50, label="current odom")

        if self.latest_goal is not None:
            p = self.latest_goal.pose.position
            ax_xy.scatter([p.x], [p.y], marker="*", s=120, label="goal")
            ax_xz.scatter([p.x], [p.z], marker="*", s=120, label="goal")
            ax_yz.scatter([p.y], [p.z], marker="*", s=120, label="goal")

        if self.latest_collision is not None and self.latest_collision.trajectory_points:
            cx = [p.x for p in self.latest_collision.trajectory_points]
            cy = [p.y for p in self.latest_collision.trajectory_points]
            cz = [p.z for p in self.latest_collision.trajectory_points]
            ax_xy.plot(cx, cy, "o--", label="collision trajectory points")
            ax_xz.plot(cx, cz, "o--", label="collision trajectory points")
            ax_yz.plot(cy, cz, "o--", label="collision trajectory points")

        if self.latest_collision is not None and self.latest_collision.collision_events:
            event = self.latest_collision.collision_events[0]
            cp = event.collision_point
            ax_xy.scatter([cp.x], [cp.y], marker="x", s=100, label="collision point")
            ax_xz.scatter([cp.x], [cp.z], marker="x", s=100, label="collision point")
            ax_yz.scatter([cp.y], [cp.z], marker="x", s=100, label="collision point")

        ax_xy.set_title("XY")
        ax_xy.set_xlabel("x [m]")
        ax_xy.set_ylabel("y [m]")
        ax_xz.set_title("XZ")
        ax_xz.set_xlabel("x [m]")
        ax_xz.set_ylabel("z [m]")
        ax_t.set_title("Trajectory CSV vs Time")
        ax_t.set_xlabel("t [s]")
        ax_t.set_ylabel("position [m]")
        ax_yz.set_title("YZ")
        ax_yz.set_xlabel("y [m]")
        ax_yz.set_ylabel("z [m]")

        for ax in [ax_xy, ax_xz, ax_t, ax_yz]:
            ax.grid(True, linestyle="--", alpha=0.3)
            ax.legend(loc="best")

        fig.suptitle("Collision Planning Result")
        fig.tight_layout()
        fig.savefig(self.output_png, dpi=160)
        plt.close(fig)
        rospy.loginfo(
            "plot_collision_plan: saved planning result to %s (stage1=%s, stage2=%s)",
            self.output_png,
            self.stage1_csv,
            self.stage2_csv,
        )
        return True

    def _timer_cb(self, _unused_event):
        if self.saved_once and not self.overwrite:
            return
        now = rospy.Time.now()
        if self.overwrite and self.last_save_time is not None:
            elapsed = (now - self.last_save_time).to_sec()
            if elapsed < self.save_period_sec and not self._traj_files_updated():
                return
        if self._render():
            self.saved_once = True
            self.last_save_time = now


def main():
    rospy.init_node("plot_collision_plan", anonymous=False)
    CollisionPlanPlotter()
    rospy.spin()


if __name__ == "__main__":
    main()
