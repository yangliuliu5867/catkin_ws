#!/usr/bin/env python3

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import CollisionTrajectory, PositionCommand


def yaw_to_quaternion(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class FakePoscmdToOdom:
    def __init__(self):
        self.cmd_topic = rospy.get_param("~cmd_topic", "/position_command")
        self.odom_topic = rospy.get_param("~odom_topic", "/visual_slam/odom")
        self.collision_topic = rospy.get_param("~collision_topic", "/complete_collision_trajectory")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.pred_pose_topic = rospy.get_param("~pred_pose_topic", "/trajectory_predicted_pose")
        self.pred_vel_topic = rospy.get_param("~pred_vel_topic", "/trajectory_predicted_velocity")
        self.pred_acc_topic = rospy.get_param("~pred_acc_topic", "/trajectory_predicted_acceleration")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.rate_hz = max(1.0, float(rospy.get_param("~rate_hz", 50.0)))
        self.start_param = rospy.get_param("~start_param", "/PlanningStart")
        self.start_yaw = float(rospy.get_param("~start_yaw", 0.0))
        self.enable_collision_handoff = bool(rospy.get_param("~enable_collision_handoff", True))
        self.collision_transition_duration = max(
            0.0, float(rospy.get_param("~collision_transition_duration", 0.15))
        )

        start = rospy.get_param(self.start_param, [0.0, 0.0, 1.0])
        if not isinstance(start, (list, tuple)) or len(start) < 3:
            raise RuntimeError(f"Invalid start parameter at {self.start_param}")

        self.pos = [float(start[0]), float(start[1]), float(start[2])]
        self.vel = [0.0, 0.0, 0.0]
        self.acc = [0.0, 0.0, 0.0]
        self.yaw = self.start_yaw
        self.lock = threading.Lock()
        self.pending_handoff = None
        self.collision_handoff_done = False

        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=1, latch=True)
        self.pred_pose_pub = rospy.Publisher(self.pred_pose_topic, PoseStamped, queue_size=1, latch=True)
        self.pred_vel_pub = rospy.Publisher(self.pred_vel_topic, Vector3Stamped, queue_size=1, latch=True)
        self.pred_acc_pub = rospy.Publisher(self.pred_acc_topic, Vector3Stamped, queue_size=1, latch=True)
        rospy.Subscriber(self.cmd_topic, PositionCommand, self._cmd_cb, queue_size=20)
        rospy.Subscriber(self.collision_topic, CollisionTrajectory, self._collision_cb, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_cb, queue_size=1)

    def _cmd_cb(self, msg: PositionCommand):
        with self.lock:
            self.pos[0] = float(msg.position.x)
            self.pos[1] = float(msg.position.y)
            self.pos[2] = float(msg.position.z)
            self.vel[0] = float(msg.velocity.x)
            self.vel[1] = float(msg.velocity.y)
            self.vel[2] = float(msg.velocity.z)
            self.acc[0] = float(msg.acceleration.x)
            self.acc[1] = float(msg.acceleration.y)
            self.acc[2] = float(msg.acceleration.z)
            self.yaw = float(msg.yaw)

    def _goal_cb(self, _unused_msg: PoseStamped):
        with self.lock:
            self.pending_handoff = None
            self.collision_handoff_done = False

    def _collision_cb(self, msg: CollisionTrajectory):
        if not self.enable_collision_handoff or not msg.collision_events:
            return

        with self.lock:
            if self.collision_handoff_done or self.pending_handoff is not None:
                rospy.loginfo(
                    "fake_poscmd_to_odom: ignoring repeated collision trajectory after handoff scheduling/execution"
                )
                return

        event = msg.collision_events[0]
        handoff_delay = max(0.0, float(event.collision_time) + self.collision_transition_duration)
        handoff_at = rospy.Time.now() + rospy.Duration.from_sec(handoff_delay)
        handoff = {
            "at": handoff_at,
            "pos": [
                float(event.collision_point.x),
                float(event.collision_point.y),
                float(event.collision_point.z),
            ],
            "vel": [0.0, 0.0, 0.0],
            "acc": [0.0, 0.0, 0.0],
        }
        handoff["yaw"] = self.yaw
        with self.lock:
            self.pending_handoff = handoff
        rospy.loginfo(
            "fake_poscmd_to_odom: schedule collision handoff after %.3fs to pos=(%.3f, %.3f, %.3f), hold vel=(%.3f, %.3f, %.3f)",
            handoff_delay,
            handoff["pos"][0],
            handoff["pos"][1],
            handoff["pos"][2],
            handoff["vel"][0],
            handoff["vel"][1],
            handoff["vel"][2],
        )

    def _apply_pending_handoff_if_needed(self, now: rospy.Time):
        with self.lock:
            if self.pending_handoff is None or now < self.pending_handoff["at"]:
                return
            handoff = self.pending_handoff
            self.pending_handoff = None
            self.collision_handoff_done = True
            self.pos[:] = handoff["pos"]
            self.vel[:] = handoff["vel"]
            self.acc[:] = handoff["acc"]
            self.yaw = handoff["yaw"]
        rospy.loginfo(
            "fake_poscmd_to_odom: applied collision handoff at pos=(%.3f, %.3f, %.3f), vel=(%.3f, %.3f, %.3f)",
            self.pos[0],
            self.pos[1],
            self.pos[2],
            self.vel[0],
            self.vel[1],
            self.vel[2],
        )

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self._apply_pending_handoff_if_needed(now)

            with self.lock:
                pos = list(self.pos)
                vel = list(self.vel)
                acc = list(self.acc)
                yaw = float(self.yaw)

            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            msg = Odometry()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id
            msg.child_frame_id = self.child_frame_id
            msg.pose.pose.position.x = pos[0]
            msg.pose.pose.position.y = pos[1]
            msg.pose.pose.position.z = pos[2]
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.twist.twist.linear.x = vel[0]
            msg.twist.twist.linear.y = vel[1]
            msg.twist.twist.linear.z = vel[2]
            msg.twist.twist.angular.z = 0.0
            self.pub.publish(msg)

            pred_pose = PoseStamped()
            pred_pose.header.stamp = now
            pred_pose.header.frame_id = self.frame_id
            pred_pose.pose = msg.pose.pose
            self.pred_pose_pub.publish(pred_pose)

            pred_vel = Vector3Stamped()
            pred_vel.header.stamp = now
            pred_vel.header.frame_id = self.frame_id
            pred_vel.vector.x = vel[0]
            pred_vel.vector.y = vel[1]
            pred_vel.vector.z = vel[2]
            self.pred_vel_pub.publish(pred_vel)

            pred_acc = Vector3Stamped()
            pred_acc.header.stamp = now
            pred_acc.header.frame_id = self.frame_id
            pred_acc.vector.x = acc[0]
            pred_acc.vector.y = acc[1]
            pred_acc.vector.z = acc[2]
            self.pred_acc_pub.publish(pred_acc)
            rate.sleep()


def main():
    rospy.init_node("fake_poscmd_to_odom", anonymous=False)
    FakePoscmdToOdom().run()


if __name__ == "__main__":
    main()
