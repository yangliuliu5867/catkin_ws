#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/CollisionTrajectory.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace {

double clampDouble(const double value, const double low, const double high) {
    return std::max(low, std::min(high, value));
}

double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double yawFromQuaternion(const geometry_msgs::Quaternion &q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::Quaternion quaternionMsgFromYaw(const double yaw) {
    geometry_msgs::Quaternion q;
    q.w = std::cos(0.5 * yaw);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(0.5 * yaw);
    return q;
}

Eigen::Vector3d pointToEigen(const geometry_msgs::Point &pt) {
    return Eigen::Vector3d(pt.x, pt.y, pt.z);
}

geometry_msgs::Point eigenToPoint(const Eigen::Vector3d &vec) {
    geometry_msgs::Point pt;
    pt.x = vec.x();
    pt.y = vec.y();
    pt.z = vec.z();
    return pt;
}

geometry_msgs::Vector3 eigenToVector(const Eigen::Vector3d &vec) {
    geometry_msgs::Vector3 out;
    out.x = vec.x();
    out.y = vec.y();
    out.z = vec.z();
    return out;
}

double evalPoly(const std::vector<double> &coeffs, const double t) {
    double value = 0.0;
    double t_pow = 1.0;
    for (double coeff : coeffs) {
        value += coeff * t_pow;
        t_pow *= t;
    }
    return value;
}

double evalPolyDerivative(const std::vector<double> &coeffs, const double t, const int deriv) {
    if (deriv <= 0) {
        return evalPoly(coeffs, t);
    }

    double value = 0.0;
    for (size_t i = static_cast<size_t>(deriv); i < coeffs.size(); ++i) {
        double scale = 1.0;
        for (int k = 0; k < deriv; ++k) {
            scale *= static_cast<double>(static_cast<int>(i) - k);
        }
        value += coeffs[i] * scale * std::pow(t, static_cast<int>(i) - deriv);
    }
    return value;
}

}  // namespace

class PolytrajPoscmdBridge {
public:
    PolytrajPoscmdBridge()
        : nh_(),
          pnh_("~") {
        loadParameters();
        setupRosInterfaces();
    }

    void spin() {
        ros::Rate rate(std::max(1.0, publish_rate_));
        while (ros::ok()) {
            ros::spinOnce();
            publishStep();
            rate.sleep();
        }
    }

private:
    struct PolySegment {
        double duration{0.0};
        int order{0};
        std::vector<double> coef_x;
        std::vector<double> coef_y;
        std::vector<double> coef_z;
    };

    struct TrajectorySample {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
        Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
        Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
        int segment_index{0};
    };

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber traj_sub_;
    ros::Subscriber collision_traj_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber start_trigger_sub_;
    ros::Subscriber ext_force_sub_;

    ros::Publisher cmd_pub_;
    ros::Publisher pred_pose_pub_;
    ros::Publisher pred_vel_pub_;
    ros::Publisher pred_acc_pub_;
    ros::Publisher remaining_pub_;
    ros::Publisher segment_pub_;
    ros::Publisher collision_trigger_pub_;

    std::string traj_topic_;
    std::string collision_traj_topic_;
    std::string odom_topic_;
    std::string start_trigger_topic_;
    std::string ext_force_topic_;
    std::string cmd_topic_;
    std::string pred_pose_topic_;
    std::string pred_vel_topic_;
    std::string pred_acc_topic_;
    std::string remaining_time_topic_;
    std::string current_segment_topic_;
    std::string collision_trigger_topic_;

    double publish_rate_{100.0};
    double traj_start_delay_{0.0};
    double collision_transition_duration_{0.15};
    double planned_collision_hold_sec_{0.15};
    double external_force_collision_threshold_{4.0};

    bool tracking_enabled_{false};
    bool has_odom_{false};
    bool hold_final_position_{true};
    bool use_external_force_collision_hold_{false};
    bool ext_force_high_{false};
    bool force_collision_hold_active_{false};
    bool waiting_post_collision_traj_{false};

    nav_msgs::Odometry latest_odom_;
    double hold_yaw_{0.0};
    Eigen::Vector3d collision_hold_position_{Eigen::Vector3d::Zero()};

    bool has_traj_{false};
    uint32_t traj_id_{0};
    ros::Time traj_start_time_{ros::TIME_MAX};
    double total_duration_{0.0};
    std::vector<PolySegment> segments_;
    TrajectorySample final_sample_;

    bool has_collision_event_{false};
    quadrotor_msgs::CollisionEvent collision_event_;
    ros::Time last_collision_msg_time_{ros::TIME_MIN};

    void loadParameters() {
        pnh_.param<std::string>("traj_topic", traj_topic_, std::string("/trajectory_generator_node/trajectory"));
        pnh_.param<std::string>("collision_traj_topic", collision_traj_topic_, std::string("/complete_collision_trajectory"));
        pnh_.param<std::string>("odom_topic", odom_topic_, std::string("/visual_slam/odom"));
        pnh_.param<std::string>("start_trigger_topic", start_trigger_topic_, std::string("/traj_start_trigger"));
        pnh_.param<std::string>("ext_force_topic", ext_force_topic_, std::string("/external_force_est"));
        pnh_.param<std::string>("cmd_topic", cmd_topic_, std::string("/position_command"));
        pnh_.param<std::string>("pred_pose_topic", pred_pose_topic_, std::string("/trajectory_predicted_pose"));
        pnh_.param<std::string>("pred_vel_topic", pred_vel_topic_, std::string("/trajectory_predicted_velocity"));
        pnh_.param<std::string>("pred_acc_topic", pred_acc_topic_, std::string("/trajectory_predicted_acceleration"));
        pnh_.param<std::string>("remaining_time_topic", remaining_time_topic_, std::string("/trajectory_remaining_time"));
        pnh_.param<std::string>("current_segment_topic", current_segment_topic_, std::string("/trajectory_current_segment"));
        pnh_.param<std::string>("collision_trigger_topic", collision_trigger_topic_, std::string("/collision_trigger"));
        pnh_.param("publish_rate", publish_rate_, publish_rate_);
        pnh_.param("traj_start_delay", traj_start_delay_, traj_start_delay_);
        pnh_.param("collision_transition_duration", collision_transition_duration_, collision_transition_duration_);
        pnh_.param("planned_collision_hold_sec", planned_collision_hold_sec_, planned_collision_hold_sec_);
        pnh_.param("hold_final_position", hold_final_position_, hold_final_position_);
        pnh_.param("use_external_force_collision_hold", use_external_force_collision_hold_, use_external_force_collision_hold_);
        pnh_.param("external_force_collision_threshold", external_force_collision_threshold_, external_force_collision_threshold_);

        collision_transition_duration_ = std::max(1e-3, collision_transition_duration_);
        planned_collision_hold_sec_ = std::max(0.0, planned_collision_hold_sec_);
        external_force_collision_threshold_ = std::max(0.0, external_force_collision_threshold_);
    }

    void setupRosInterfaces() {
        traj_sub_ = nh_.subscribe(traj_topic_, 2, &PolytrajPoscmdBridge::trajCallback, this);
        collision_traj_sub_ = nh_.subscribe(collision_traj_topic_, 2, &PolytrajPoscmdBridge::collisionTrajCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 20, &PolytrajPoscmdBridge::odomCallback, this, ros::TransportHints().tcpNoDelay());
        start_trigger_sub_ = nh_.subscribe(start_trigger_topic_, 10, &PolytrajPoscmdBridge::startTriggerCallback, this, ros::TransportHints().tcpNoDelay());
        ext_force_sub_ = nh_.subscribe(ext_force_topic_, 20, &PolytrajPoscmdBridge::extForceCallback, this, ros::TransportHints().tcpNoDelay());

        cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(cmd_topic_, 20);
        pred_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pred_pose_topic_, 1);
        pred_vel_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(pred_vel_topic_, 1);
        pred_acc_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(pred_acc_topic_, 1);
        remaining_pub_ = nh_.advertise<std_msgs::Float64>(remaining_time_topic_, 1);
        segment_pub_ = nh_.advertise<std_msgs::Int32>(current_segment_topic_, 1);
        collision_trigger_pub_ = nh_.advertise<std_msgs::Bool>(collision_trigger_topic_, 1);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        latest_odom_ = *msg;
        has_odom_ = true;
    }

    void startTriggerCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        tracking_enabled_ = (msg->header.frame_id == "start");
        if (tracking_enabled_) {
            if (has_odom_) {
                hold_yaw_ = yawFromQuaternion(latest_odom_.pose.pose.orientation);
            } else {
                hold_yaw_ = yawFromQuaternion(msg->pose.orientation);
            }
        }
    }

    void extForceCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        if (!use_external_force_collision_hold_) {
            return;
        }

        const Eigen::Vector3d f(msg->vector.x, msg->vector.y, msg->vector.z);
        const double force_norm = f.norm();
        const bool force_high = force_norm > external_force_collision_threshold_;

        if (force_high && !ext_force_high_) {
            ext_force_high_ = true;
            force_collision_hold_active_ = true;
            waiting_post_collision_traj_ = false;
            if (has_odom_) {
                collision_hold_position_.x() = latest_odom_.pose.pose.position.x;
                collision_hold_position_.y() = latest_odom_.pose.pose.position.y;
                collision_hold_position_.z() = latest_odom_.pose.pose.position.z;
                hold_yaw_ = yawFromQuaternion(latest_odom_.pose.pose.orientation);
            } else {
                collision_hold_position_.setZero();
            }
            ROS_WARN("[polytraj_poscmd_bridge] External force %.3fN > %.3fN, enter collision hold",
                     force_norm, external_force_collision_threshold_);
        } else if (!force_high && ext_force_high_) {
            ext_force_high_ = false;
            if (force_collision_hold_active_) {
                waiting_post_collision_traj_ = true;
                ROS_INFO("[polytraj_poscmd_bridge] External force %.3fN < %.3fN, keep hold and wait for post-collision trajectory",
                         force_norm, external_force_collision_threshold_);
            }
        }
    }

    void collisionTrajCallback(const quadrotor_msgs::CollisionTrajectory::ConstPtr &msg) {
        has_collision_event_ = false;
        if (msg->collision_events.empty()) {
            return;
        }

        const auto &event = msg->collision_events.front();
        if (!std::isfinite(event.collision_time) || event.collision_time < 0.0) {
            ROS_WARN_THROTTLE(1.0, "[polytraj_poscmd_bridge] Ignore collision event with invalid collision_time");
            return;
        }

        collision_event_ = event;
        has_collision_event_ = true;
        last_collision_msg_time_ = ros::Time::now();
    }

    void trajCallback(const quadrotor_msgs::PolynomialTrajectory::ConstPtr &msg) {
        if (msg->action != quadrotor_msgs::PolynomialTrajectory::ACTION_ADD) {
            if (msg->action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT) {
                has_traj_ = false;
            }
            return;
        }

        if (msg->num_segment == 0 || msg->time.size() != msg->num_segment || msg->order.size() != msg->num_segment) {
            ROS_ERROR("[polytraj_poscmd_bridge] Invalid trajectory segment metadata");
            return;
        }

        std::vector<PolySegment> parsed_segments;
        parsed_segments.reserve(msg->num_segment);

        size_t shift = 0;
        for (size_t idx = 0; idx < msg->num_segment; ++idx) {
            const int order = static_cast<int>(msg->order[idx]);
            if (order < 0) {
                ROS_ERROR("[polytraj_poscmd_bridge] Negative polynomial order");
                return;
            }

            const size_t coeff_count = static_cast<size_t>(order + 1);
            if (shift + coeff_count > msg->coef_x.size() ||
                shift + coeff_count > msg->coef_y.size() ||
                shift + coeff_count > msg->coef_z.size()) {
                ROS_ERROR("[polytraj_poscmd_bridge] Trajectory coefficient size mismatch");
                return;
            }

            PolySegment seg;
            seg.duration = std::max(1e-6, msg->time[idx]);
            seg.order = order;
            seg.coef_x.assign(msg->coef_x.begin() + static_cast<long>(shift),
                              msg->coef_x.begin() + static_cast<long>(shift + coeff_count));
            seg.coef_y.assign(msg->coef_y.begin() + static_cast<long>(shift),
                              msg->coef_y.begin() + static_cast<long>(shift + coeff_count));
            seg.coef_z.assign(msg->coef_z.begin() + static_cast<long>(shift),
                              msg->coef_z.begin() + static_cast<long>(shift + coeff_count));
            parsed_segments.push_back(std::move(seg));
            shift += coeff_count;
        }

        segments_ = std::move(parsed_segments);
        total_duration_ = std::accumulate(
            segments_.begin(), segments_.end(), 0.0,
            [](const double sum, const PolySegment &seg) { return sum + seg.duration; });

        traj_id_ = msg->trajectory_id;
        const ros::Time now = ros::Time::now();
        if (last_collision_msg_time_ == ros::TIME_MIN || (now - last_collision_msg_time_).toSec() > 1.0) {
            has_collision_event_ = false;
        }
        if (!msg->header.stamp.isZero() && msg->header.stamp > now) {
            traj_start_time_ = msg->header.stamp;
        } else {
            traj_start_time_ = now + ros::Duration(traj_start_delay_);
        }

        final_sample_ = sampleBaseTrajectory(total_duration_);
        has_traj_ = true;

        if (use_external_force_collision_hold_ && waiting_post_collision_traj_) {
            force_collision_hold_active_ = false;
            waiting_post_collision_traj_ = false;
            traj_start_time_ = now + ros::Duration(traj_start_delay_);
            ROS_INFO("[polytraj_poscmd_bridge] Received post-collision trajectory, exit collision hold");
        }

        if (has_collision_event_ && collision_event_.collision_time > total_duration_ + 1e-3) {
            ROS_WARN("[polytraj_poscmd_bridge] Collision time %.3f exceeds total duration %.3f, ignore collision blending",
                     collision_event_.collision_time, total_duration_);
            has_collision_event_ = false;
        }
    }

    TrajectorySample sampleBaseTrajectory(const double t_global) const {
        TrajectorySample sample;
        if (segments_.empty()) {
            return sample;
        }

        const double t_clamped = clampDouble(t_global, 0.0, total_duration_);
        double accum = 0.0;
        int seg_idx = static_cast<int>(segments_.size()) - 1;
        double t_local = segments_.back().duration;

        for (size_t i = 0; i < segments_.size(); ++i) {
            const double seg_end = accum + segments_[i].duration;
            if (t_clamped <= seg_end || i + 1 == segments_.size()) {
                seg_idx = static_cast<int>(i);
                t_local = clampDouble(t_clamped - accum, 0.0, segments_[i].duration);
                break;
            }
            accum = seg_end;
        }

        const PolySegment &seg = segments_[static_cast<size_t>(seg_idx)];
        const double seg_time = std::max(1e-6, seg.duration);
        const double tau = clampDouble(t_local / seg_time, 0.0, 1.0);
        sample.segment_index = seg_idx;
        sample.position.x() = evalPoly(seg.coef_x, tau);
        sample.position.y() = evalPoly(seg.coef_y, tau);
        sample.position.z() = evalPoly(seg.coef_z, tau);
        sample.velocity.x() = evalPolyDerivative(seg.coef_x, tau, 1) / seg_time;
        sample.velocity.y() = evalPolyDerivative(seg.coef_y, tau, 1) / seg_time;
        sample.velocity.z() = evalPolyDerivative(seg.coef_z, tau, 1) / seg_time;
        sample.acceleration.x() = evalPolyDerivative(seg.coef_x, tau, 2) / (seg_time * seg_time);
        sample.acceleration.y() = evalPolyDerivative(seg.coef_y, tau, 2) / (seg_time * seg_time);
        sample.acceleration.z() = evalPolyDerivative(seg.coef_z, tau, 2) / (seg_time * seg_time);
        sample.jerk.x() = evalPolyDerivative(seg.coef_x, tau, 3) / (seg_time * seg_time * seg_time);
        sample.jerk.y() = evalPolyDerivative(seg.coef_y, tau, 3) / (seg_time * seg_time * seg_time);
        sample.jerk.z() = evalPolyDerivative(seg.coef_z, tau, 3) / (seg_time * seg_time * seg_time);
        return sample;
    }

    TrajectorySample sampleCollisionBlend(const double t_global) const {
        TrajectorySample sample;
        sample.segment_index = sampleBaseTrajectory(collision_event_.collision_time).segment_index;

        const double tau = clampDouble(t_global - collision_event_.collision_time, 0.0, collision_transition_duration_);
        const Eigen::Vector3d p0 = pointToEigen(collision_event_.collision_point);
        const Eigen::Vector3d v0 = Eigen::Vector3d(
            collision_event_.pre_collision_velocity.x,
            collision_event_.pre_collision_velocity.y,
            collision_event_.pre_collision_velocity.z);
        const Eigen::Vector3d v1 = Eigen::Vector3d(
            collision_event_.post_collision_velocity.x,
            collision_event_.post_collision_velocity.y,
            collision_event_.post_collision_velocity.z);

        const Eigen::Vector3d acc = (v1 - v0) / collision_transition_duration_;
        sample.position = p0 + v0 * tau + 0.5 * acc * tau * tau;
        sample.velocity = v0 + acc * tau;
        sample.acceleration = acc;
        sample.jerk = Eigen::Vector3d::Zero();
        return sample;
    }

    double collisionEndTime() const {
        return collision_event_.collision_time + collision_transition_duration_;
    }

    bool useCollisionBlend(const double t_global) const {
        if (use_external_force_collision_hold_) {
            return false;
        }
        if (!has_collision_event_) {
            return false;
        }
        return t_global >= collision_event_.collision_time &&
               t_global <= collision_event_.collision_time + collision_transition_duration_;
    }

    TrajectorySample sampleCollisionHold() const {
        TrajectorySample sample;
        sample.position = collision_hold_position_;
        sample.velocity.setZero();
        sample.acceleration.setZero();
        sample.jerk.setZero();
        sample.segment_index = 0;
        return sample;
    }

    void publishStep() {
        if (!tracking_enabled_ || !has_traj_) {
            return;
        }

        const ros::Time now = ros::Time::now();
        double t_from_start = 0.0;
        if (traj_start_time_ != ros::TIME_MAX && !traj_start_time_.isZero()) {
            t_from_start = (now - traj_start_time_).toSec();
        }

        const bool before_start = (t_from_start < 0.0);
        const double t_eval = before_start ? 0.0 : t_from_start;
        const bool completed = (t_eval >= total_duration_);

        TrajectorySample sample;
        if (use_external_force_collision_hold_ && force_collision_hold_active_) {
            sample = sampleCollisionHold();
        } else if (completed && hold_final_position_) {
            sample = final_sample_;
            sample.velocity.setZero();
            sample.acceleration.setZero();
            sample.jerk.setZero();
            sample.segment_index = std::max(0, static_cast<int>(segments_.size()) - 1);
        } else if (useCollisionBlend(t_eval)) {
            sample = sampleCollisionBlend(t_eval);
        } else {
            sample = sampleBaseTrajectory(t_eval);
        }

        double cmd_yaw = normalizeAngle(hold_yaw_);
        if (use_external_force_collision_hold_ && force_collision_hold_active_ && has_odom_) {
            cmd_yaw = normalizeAngle(yawFromQuaternion(latest_odom_.pose.pose.orientation));
        }

        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = now;
        cmd.header.frame_id = "world";
        cmd.position = eigenToPoint(sample.position);
        cmd.velocity = eigenToVector(sample.velocity);
        cmd.acceleration = eigenToVector(sample.acceleration);
        cmd.jerk = eigenToVector(sample.jerk);
        cmd.yaw = cmd_yaw;
        cmd.yaw_dot = 0.0;
        cmd.trajectory_id = traj_id_;
        cmd.trajectory_flag = completed ? quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED
                                        : quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        cmd_pub_.publish(cmd);

        publishStatus(now, t_eval, sample);
    }

    void publishStatus(const ros::Time &stamp, const double t_eval, const TrajectorySample &sample) {
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = "world";
        ps.pose.position = eigenToPoint(sample.position);
        double vis_yaw = normalizeAngle(hold_yaw_);
        if (use_external_force_collision_hold_ && force_collision_hold_active_ && has_odom_) {
            vis_yaw = normalizeAngle(yawFromQuaternion(latest_odom_.pose.pose.orientation));
        }
        ps.pose.orientation = quaternionMsgFromYaw(vis_yaw);
        pred_pose_pub_.publish(ps);

        geometry_msgs::Vector3Stamped vs;
        vs.header.stamp = stamp;
        vs.header.frame_id = "world";
        vs.vector = eigenToVector(sample.velocity);
        pred_vel_pub_.publish(vs);

        geometry_msgs::Vector3Stamped as;
        as.header.stamp = stamp;
        as.header.frame_id = "world";
        as.vector = eigenToVector(sample.acceleration);
        pred_acc_pub_.publish(as);

        std_msgs::Float64 rem;
        rem.data = std::max(0.0, total_duration_ - clampDouble(t_eval, 0.0, total_duration_));
        remaining_pub_.publish(rem);

        std_msgs::Int32 seg;
        seg.data = sample.segment_index;
        segment_pub_.publish(seg);

        std_msgs::Bool trig;
        // Publish the "collision finished" window after the planned velocity blend
        // completes, not at the instant the collision starts.
        const double collision_end_time = has_collision_event_ ? collisionEndTime() : 0.0;
        trig.data = (!use_external_force_collision_hold_) &&
                    has_collision_event_ &&
                    t_eval >= collision_end_time &&
                    t_eval <= collision_end_time + planned_collision_hold_sec_;
        collision_trigger_pub_.publish(trig);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "polytraj_poscmd_bridge");
    PolytrajPoscmdBridge node;
    node.spin();
    return 0;
}
