#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <string>

#include <Eigen/Dense>

namespace {

Eigen::Vector3d vec3ToEigen(const geometry_msgs::Vector3 &v) {
    return Eigen::Vector3d(v.x, v.y, v.z);
}

geometry_msgs::Vector3 eigenToVec3(const Eigen::Vector3d &v) {
    geometry_msgs::Vector3 out;
    out.x = v.x();
    out.y = v.y();
    out.z = v.z();
    return out;
}

Eigen::Vector3d pointToEigen(const geometry_msgs::Point &p) {
    return Eigen::Vector3d(p.x, p.y, p.z);
}

geometry_msgs::Point eigenToPoint(const Eigen::Vector3d &p) {
    geometry_msgs::Point out;
    out.x = p.x();
    out.y = p.y();
    out.z = p.z();
    return out;
}

double clampAbs(const double value, const double max_abs) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    if (max_abs <= 0.0) {
        return value;
    }
    return std::max(-max_abs, std::min(max_abs, value));
}

Eigen::Vector3d clampNorm(const Eigen::Vector3d &v, const double max_norm) {
    if (max_norm <= 0.0) {
        return v;
    }
    const double n = v.norm();
    if (!std::isfinite(n) || n < 1e-9) {
        return Eigen::Vector3d::Zero();
    }
    if (n <= max_norm) {
        return v;
    }
    return v * (max_norm / n);
}

}  // namespace

class PoscmdAdmittanceOverlay {
public:
    PoscmdAdmittanceOverlay() : nh_(), pnh_("~") {
        loadParams();
        setupRos();
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber ref_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber force_sub_;
    ros::Publisher out_pub_;
    ros::Timer timer_;

    std::string ref_topic_;
    std::string odom_topic_;
    std::string force_topic_;
    std::string out_topic_;

    double publish_rate_{100.0};

    // Collision gating on external force magnitude
    double force_threshold_{2.0};
    int enter_confirm_count_{3};
    int release_confirm_count_{10};
    double max_dt_{0.05};
    double collision_holdoff_after_traj_start_sec_{1.0};

    // Safety clamps (relative to reference)
    double max_position_offset_{0.30};
    double max_velocity_offset_{2.0};
    double max_acceleration_offset_{10.0};
    double max_external_force_norm_{50.0};

    bool passthrough_without_force_{true};

    // Admittance params (diagonal)
    Eigen::Matrix3d M_{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d D_{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d K_{Eigen::Matrix3d::Zero()};
    Eigen::Matrix3d M_inv_{Eigen::Matrix3d::Identity()};

    // Latest inputs
    quadrotor_msgs::PositionCommand latest_ref_;
    bool has_ref_{false};
    uint32_t last_ref_trajectory_id_{0};
    std::atomic<double> collision_holdoff_until_sec_{0.0};

    nav_msgs::Odometry latest_odom_;
    bool has_odom_{false};

    Eigen::Vector3d F_ext_{Eigen::Vector3d::Zero()};
    bool has_force_{false};

    // Collision state
    bool in_collision_{false};
    int high_force_count_{0};
    int low_force_count_{0};

    // Admittance internal state (world)
    bool adm_init_{true};
    Eigen::Vector3d p_rf_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_rf_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a_rf_{Eigen::Vector3d::Zero()};
    ros::Time last_update_{ros::TIME_MIN};

    void loadParams() {
        pnh_.param<std::string>("ref_topic", ref_topic_, std::string("/position_command_ref"));
        pnh_.param<std::string>("odom_topic", odom_topic_, std::string("/visual_slam/odom"));
        pnh_.param<std::string>("force_topic", force_topic_, std::string("/external_force_est"));
        pnh_.param<std::string>("out_topic", out_topic_, std::string("/position_command"));

        pnh_.param("publish_rate", publish_rate_, publish_rate_);
        pnh_.param("force_threshold", force_threshold_, force_threshold_);
        pnh_.param("enter_confirm_count", enter_confirm_count_, enter_confirm_count_);
        pnh_.param("release_confirm_count", release_confirm_count_, release_confirm_count_);
        pnh_.param("max_dt", max_dt_, max_dt_);

        pnh_.param("max_position_offset", max_position_offset_, max_position_offset_);
        pnh_.param("max_velocity_offset", max_velocity_offset_, max_velocity_offset_);
        pnh_.param("max_acceleration_offset", max_acceleration_offset_, max_acceleration_offset_);
        pnh_.param("max_external_force_norm", max_external_force_norm_, max_external_force_norm_);
        pnh_.param("collision_force_ignore_after_traj_start_sec", collision_holdoff_after_traj_start_sec_, 1.0);

        pnh_.param("passthrough_without_force", passthrough_without_force_, passthrough_without_force_);

        // Reuse the same param names as collision_recovery/admittance nodes.
        pnh_.param("AdmittanceControl/M_vx", M_(0, 0), M_(0, 0));
        pnh_.param("AdmittanceControl/M_vy", M_(1, 1), M_(1, 1));
        pnh_.param("AdmittanceControl/M_vz", M_(2, 2), M_(2, 2));
        pnh_.param("AdmittanceControl/D_vx", D_(0, 0), D_(0, 0));
        pnh_.param("AdmittanceControl/D_vy", D_(1, 1), D_(1, 1));
        pnh_.param("AdmittanceControl/D_vz", D_(2, 2), D_(2, 2));
        pnh_.param("AdmittanceControl/K_vx", K_(0, 0), K_(0, 0));
        pnh_.param("AdmittanceControl/K_vy", K_(1, 1), K_(1, 1));
        pnh_.param("AdmittanceControl/K_vz", K_(2, 2), K_(2, 2));

        // If user loads px4ctrl/config/collision_recovery.yaml, also honor its threshold field.
        // Only apply when force_threshold is not explicitly set (>0) by overlay params.
        double f_min = force_threshold_;
        if (pnh_.getParam("Threshold/F_collision_min", f_min)) {
            if (!(force_threshold_ > 0.0)) {
                force_threshold_ = f_min;
            }
        }

        // Validate M diagonal for inversion
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(M_(i, i)) || M_(i, i) <= 1e-6) {
                ROS_WARN("[poscmd_admittance_overlay] Invalid M diag at %d (%.6f), fallback to 1.0", i, M_(i, i));
                M_(i, i) = 1.0;
            }
        }
        M_inv_ = M_.inverse();
        publish_rate_ = std::max(1.0, publish_rate_);
        enter_confirm_count_ = std::max(1, enter_confirm_count_);
        release_confirm_count_ = std::max(1, release_confirm_count_);
        max_dt_ = std::max(1e-3, max_dt_);
    }

    void setupRos() {
        ref_sub_ = nh_.subscribe(ref_topic_, 20, &PoscmdAdmittanceOverlay::refCb, this, ros::TransportHints().tcpNoDelay());
        odom_sub_ = nh_.subscribe(odom_topic_, 20, &PoscmdAdmittanceOverlay::odomCb, this, ros::TransportHints().tcpNoDelay());
        force_sub_ = nh_.subscribe(force_topic_, 20, &PoscmdAdmittanceOverlay::forceCb, this, ros::TransportHints().tcpNoDelay());

        out_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(out_topic_, 20);

        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &PoscmdAdmittanceOverlay::onTimer, this);
    }

    void refCb(const quadrotor_msgs::PositionCommand::ConstPtr &msg) {
        if (msg->trajectory_id != last_ref_trajectory_id_) {
            last_ref_trajectory_id_ = msg->trajectory_id;
            if (msg->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY) {
                collision_holdoff_until_sec_.store(ros::Time::now().toSec() + std::max(0.0, collision_holdoff_after_traj_start_sec_));
                ROS_INFO_THROTTLE(1.0, "[poscmd_admittance_overlay] New trajectory detected, suppress collision judge until %.3f",
                                  collision_holdoff_until_sec_.load());
            }
        }
        latest_ref_ = *msg;
        has_ref_ = true;
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr &msg) {
        latest_odom_ = *msg;
        has_odom_ = true;
    }

    void forceCb(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        Eigen::Vector3d f(msg->vector.x, msg->vector.y, msg->vector.z);
        if (f.array().isNaN().any()) {
            return;
        }
        f.z() = 0.0;
        F_ext_ = clampNorm(f, max_external_force_norm_);
        has_force_ = true;
    }

    void enterCollision(const ros::Time &now) {
        in_collision_ = true;
        adm_init_ = true;
        last_update_ = now;
        ROS_WARN_THROTTLE(1.0, "[poscmd_admittance_overlay] Enter collision/admittance mode (|F_xy|=%.3f)", std::hypot(F_ext_.x(), F_ext_.y()));
    }

    void exitCollision(const ros::Time &now) {
        (void)now;
        in_collision_ = false;
        adm_init_ = true;
        ROS_INFO_THROTTLE(1.0, "[poscmd_admittance_overlay] Exit collision/admittance mode");
    }

    void updateCollisionGate() {
        const double now_sec = ros::Time::now().toSec();
        if (now_sec < collision_holdoff_until_sec_.load()) {
            return;
        }

        const double force_xy = std::hypot(F_ext_.x(), F_ext_.y());
        const bool force_high = has_force_ && (force_xy >= force_threshold_);

        if (force_high) {
            high_force_count_++;
            low_force_count_ = 0;
        } else {
            low_force_count_++;
            high_force_count_ = 0;
        }

        const ros::Time now = ros::Time::now();
        if (!in_collision_ && force_high && high_force_count_ >= enter_confirm_count_) {
            enterCollision(now);
        }
        if (in_collision_ && !force_high && low_force_count_ >= release_confirm_count_) {
            exitCollision(now);
        }
    }

    void onTimer(const ros::TimerEvent &) {
        if (!has_ref_) {
            return;
        }

        if (!has_force_ && !passthrough_without_force_) {
            return;
        }

        updateCollisionGate();

        if (!in_collision_ || !has_force_) {
            // Pure polynomial tracking: forward the reference cmd.
            quadrotor_msgs::PositionCommand out = latest_ref_;
            out.header.stamp = ros::Time::now();
            out_pub_.publish(out);
            return;
        }

        const ros::Time now = ros::Time::now();
        double dt = (now - last_update_).toSec();
        if (!std::isfinite(dt) || dt <= 0.0) {
            dt = 1.0 / publish_rate_;
        }
        dt = std::min(dt, max_dt_);

        const Eigen::Vector3d p_ref = pointToEigen(latest_ref_.position);
        const Eigen::Vector3d v_ref = vec3ToEigen(latest_ref_.velocity);
        const Eigen::Vector3d a_ref = vec3ToEigen(latest_ref_.acceleration);

        if (adm_init_) {
            adm_init_ = false;
            if (has_odom_) {
                p_rf_ = pointToEigen(latest_odom_.pose.pose.position);
                v_rf_ = vec3ToEigen(latest_odom_.twist.twist.linear);
            } else {
                p_rf_ = p_ref;
                v_rf_ = v_ref;
            }
            a_rf_ = a_ref;
        }

        // Admittance around polynomial reference: compliant offset driven by external force.
        // a_rf = M^-1 * F_ext + a_ref - M^-1 D (v_rf - v_ref) - M^-1 K (p_rf - p_ref)
        Eigen::Vector3d a_rf = M_inv_ * F_ext_ + a_ref;
        a_rf -= M_inv_ * D_ * (v_rf_ - v_ref);
        a_rf -= M_inv_ * K_ * (p_rf_ - p_ref);

        // Integrate
        const Eigen::Vector3d v_prev = v_rf_;
        v_rf_ = v_rf_ + a_rf * dt;
        p_rf_ = p_rf_ + v_prev * dt + 0.5 * a_rf * dt * dt;
        a_rf_ = a_rf;

        // Clamp relative offsets to keep behavior bounded.
        Eigen::Vector3d dp = p_rf_ - p_ref;
        dp.x() = clampAbs(dp.x(), max_position_offset_);
        dp.y() = clampAbs(dp.y(), max_position_offset_);
        dp.z() = clampAbs(dp.z(), max_position_offset_);
        p_rf_ = p_ref + dp;

        Eigen::Vector3d dv = v_rf_ - v_ref;
        dv = clampNorm(dv, max_velocity_offset_);
        v_rf_ = v_ref + dv;

        Eigen::Vector3d da = a_rf_ - a_ref;
        da = clampNorm(da, max_acceleration_offset_);
        a_rf_ = a_ref + da;

        quadrotor_msgs::PositionCommand out = latest_ref_;
        out.header.stamp = now;
        out.position = eigenToPoint(p_rf_);
        out.velocity = eigenToVec3(v_rf_);
        out.acceleration = eigenToVec3(a_rf_);
        out_pub_.publish(out);

        last_update_ = now;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "poscmd_admittance_overlay");
    PoscmdAdmittanceOverlay node;
    ros::spin();
    return 0;
}
