#!/usr/bin/env python3

import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rospy


def _load_reference_csv(path: str):
    if not path or not os.path.exists(path):
        return None

    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                rows.append(
                    {
                        "t": float(row["t_sec"]),
                        "x": float(row["x"]),
                        "y": float(row["y"]),
                        "z": float(row["z"]),
                        "vx": float(row["vx"]),
                        "vy": float(row["vy"]),
                        "vz": float(row["vz"]),
                        "ax": float(row["ax"]),
                        "ay": float(row["ay"]),
                        "az": float(row["az"]),
                        "yaw": float(row["yaw"]),
                    }
                )
            except (KeyError, ValueError):
                continue
    return rows if len(rows) >= 2 else None


class CollisionReferencePlotter:
    def __init__(self):
        self.input_csv = rospy.get_param(
            "~input_csv",
            "/home/liet/Catkin_Ego/Aerobatic-Planner-main/real_coll/data_/collision_reference.csv",
        )
        self.output_png = rospy.get_param(
            "~output_png",
            "/home/liet/Catkin_Ego/Aerobatic-Planner-main/real_coll/data_/collision_reference.png",
        )
        self.save_period_sec = max(0.2, float(rospy.get_param("~save_period_sec", 1.0)))
        self.last_save_time = None
        self.last_mtime = None

        rospy.Timer(rospy.Duration.from_sec(0.5), self._timer_cb)

    def _render(self):
        rows = _load_reference_csv(self.input_csv)
        if rows is None:
            return False

        os.makedirs(os.path.dirname(self.output_png), exist_ok=True)

        t = [r["t"] for r in rows]
        x = [r["x"] for r in rows]
        y = [r["y"] for r in rows]
        z = [r["z"] for r in rows]
        vx = [r["vx"] for r in rows]
        vy = [r["vy"] for r in rows]
        vz = [r["vz"] for r in rows]
        ax = [r["ax"] for r in rows]
        ay = [r["ay"] for r in rows]
        az = [r["az"] for r in rows]
        yaw = [r["yaw"] for r in rows]

        fig, axes = plt.subplots(2, 3, figsize=(15, 9))
        ax_xy = axes[0][0]
        ax_xz = axes[0][1]
        ax_yz = axes[0][2]
        ax_p = axes[1][0]
        ax_v = axes[1][1]
        ax_a = axes[1][2]

        ax_xy.plot(x, y, linewidth=2.0, label="reference")
        ax_xy.scatter([x[0]], [y[0]], marker="o", s=60, label="start")
        ax_xy.scatter([x[-1]], [y[-1]], marker="*", s=120, label="end")

        ax_xz.plot(x, z, linewidth=2.0, label="reference")
        ax_xz.scatter([x[0]], [z[0]], marker="o", s=60, label="start")
        ax_xz.scatter([x[-1]], [z[-1]], marker="*", s=120, label="end")

        ax_yz.plot(y, z, linewidth=2.0, label="reference")
        ax_yz.scatter([y[0]], [z[0]], marker="o", s=60, label="start")
        ax_yz.scatter([y[-1]], [z[-1]], marker="*", s=120, label="end")

        ax_p.plot(t, x, label="x")
        ax_p.plot(t, y, label="y")
        ax_p.plot(t, z, label="z")

        ax_v.plot(t, vx, label="vx")
        ax_v.plot(t, vy, label="vy")
        ax_v.plot(t, vz, label="vz")

        ax_a.plot(t, ax, label="ax")
        ax_a.plot(t, ay, label="ay")
        ax_a.plot(t, az, label="az")
        ax_a.plot(t, yaw, label="yaw", linestyle="--")

        ax_xy.set_title("XY")
        ax_xy.set_xlabel("x [m]")
        ax_xy.set_ylabel("y [m]")
        ax_xz.set_title("XZ")
        ax_xz.set_xlabel("x [m]")
        ax_xz.set_ylabel("z [m]")
        ax_yz.set_title("YZ")
        ax_yz.set_xlabel("y [m]")
        ax_yz.set_ylabel("z [m]")
        ax_p.set_title("Position vs Time")
        ax_p.set_xlabel("t [s]")
        ax_p.set_ylabel("position [m]")
        ax_v.set_title("Velocity vs Time")
        ax_v.set_xlabel("t [s]")
        ax_v.set_ylabel("velocity [m/s]")
        ax_a.set_title("Acceleration / Yaw vs Time")
        ax_a.set_xlabel("t [s]")
        ax_a.set_ylabel("acc / yaw")

        for ax in [ax_xy, ax_xz, ax_yz, ax_p, ax_v, ax_a]:
            ax.grid(True, linestyle="--", alpha=0.3)
            ax.legend(loc="best")

        fig.suptitle("Collision Reference Trajectory")
        fig.tight_layout()
        fig.savefig(self.output_png, dpi=160)
        plt.close(fig)
        rospy.loginfo("plot_collision_reference: saved reference figure to %s", self.output_png)
        return True

    def _timer_cb(self, _unused_event):
        if not os.path.exists(self.input_csv):
            return
        mtime = os.path.getmtime(self.input_csv)
        now = rospy.Time.now()
        if self.last_mtime == mtime and self.last_save_time is not None:
            if (now - self.last_save_time).to_sec() < self.save_period_sec:
                return
            return
        if self._render():
            self.last_mtime = mtime
            self.last_save_time = now


def main():
    rospy.init_node("plot_collision_reference", anonymous=False)
    CollisionReferencePlotter()
    rospy.spin()


if __name__ == "__main__":
    main()
