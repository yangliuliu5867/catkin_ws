#include <fsm_ctrl/px4ctrl_hover.hpp>
#include "controller.h"  // Original catkin_ws/src/px4ctrl/src/controller.h
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
bool fresh(const ros::Time &stamp, const ros::Time &now, double timeout)
{
    const double age = (now - stamp).toSec();
    return !stamp.isZero() && age >= 0.0 && age < timeout;
}
bool finite(const geometry_msgs::Vector3 &v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
Eigen::Quaterniond quaternion(const geometry_msgs::Quaternion &q)
{
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}
bool valid(const Eigen::Quaterniond &q)
{
    return q.coeffs().allFinite() && std::isfinite(q.norm()) && q.norm() > 0.5;
}
void setAttitude(mavros_msgs::AttitudeTarget &out, const Eigen::Quaterniond &q,
                 double thrust, const ros::Time &now)
{
    out = mavros_msgs::AttitudeTarget();
    out.header.stamp = now;
    out.header.frame_id = "FCU";
    out.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    out.orientation.w = q.w(); out.orientation.x = q.x();
    out.orientation.y = q.y(); out.orientation.z = q.z();
    out.thrust = thrust;
}
}  // namespace

struct Px4CtrlHover::Impl
{
    Odom_Data_t odom;
    Imu_Data_t imu;
    ros::Time pose_stamp, velocity_stamp, imu_stamp;
    std::unique_ptr<LinearControl> controller;
};

Px4CtrlHover::Px4CtrlHover(const ros::NodeHandle &params) : impl_(new Impl)
{
    // input.cpp uses this same original global Parameter_t for the IMU LPF.
    param.config_from_ros_handle(params);
    if (param.use_bodyrate_ctrl || param.use_bridge_forwarding ||
        (param.ctrl_mode != 0 && param.ctrl_mode != 1))
        throw std::runtime_error("cmd=3/cmd=5 require px4ctrl attitude control (ctrl_mode 0/1, no forwarding)");
    if (!std::isfinite(param.thr_map.hover_percentage) || param.thr_map.hover_percentage <= 0.0 ||
        param.thr_map.hover_percentage >= 1.0 || !std::isfinite(param.gra) || param.gra <= 0.0)
        throw std::runtime_error("Invalid px4ctrl gravity/hover_percentage");
    impl_->odom.p.setZero(); impl_->odom.v.setZero(); impl_->odom.w.setZero();
    impl_->imu.q.setIdentity();
    reset();
}

Px4CtrlHover::~Px4CtrlHover() = default;

void Px4CtrlHover::reset()
{
    // A new instance also clears old thrust samples when re-entering controller modes.
    impl_->controller.reset(new LinearControl(param));
}

void Px4CtrlHover::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    impl_->pose_stamp = ros::Time(0);
    const auto &p = msg->pose.position;
    auto q = quaternion(msg->pose.orientation);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !valid(q)) return;
    impl_->odom.p = Eigen::Vector3d(p.x, p.y, p.z);
    impl_->odom.q = q.normalized();
    impl_->pose_stamp = msg->header.stamp;
}

void Px4CtrlHover::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    impl_->velocity_stamp = ros::Time(0);
    if (!finite(msg->twist.linear)) return;
    const auto &v = msg->twist.linear;
    // /mavros/local_position/velocity_local is world-frame, as px4ctrl expects.
    impl_->odom.v = Eigen::Vector3d(v.x, v.y, v.z);
    impl_->velocity_stamp = msg->header.stamp;
}

void Px4CtrlHover::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    impl_->imu_stamp = ros::Time(0);
    if (!valid(quaternion(msg->orientation)) || !finite(msg->linear_acceleration) ||
        !finite(msg->angular_velocity)) return;
    impl_->imu.feed(msg);  // Original filtering and input conversion.
    impl_->imu.q.normalize();
    impl_->imu_stamp = msg->header.stamp;
}

bool Px4CtrlHover::ready(const ros::Time &now) const
{
    return fresh(impl_->pose_stamp, now, param.msg_timeout.odom) &&
           fresh(impl_->velocity_stamp, now, param.msg_timeout.odom) &&
           fresh(impl_->imu_stamp, now, param.msg_timeout.imu);
}

mavros_msgs::AttitudeTarget Px4CtrlHover::idle(const ros::Time &now) const
{
    mavros_msgs::AttitudeTarget out;
    setAttitude(out, impl_->imu.q, 0.02, now);  // Same low idle thrust as baseline cmd=1.
    return out;
}

bool Px4CtrlHover::calculate(const geometry_msgs::Pose &target, const ros::Time &now,
                           bool estimate_thrust, mavros_msgs::AttitudeTarget &output,
                           quadrotor_msgs::Px4ctrlDebug &debug)
{
    geometry_msgs::Vector3 zero;
    return calculate(target, zero, zero, now, estimate_thrust, output, debug);
}

bool Px4CtrlHover::calculate(const geometry_msgs::Pose &target,
                           const geometry_msgs::Vector3 &velocity,
                           const geometry_msgs::Vector3 &acceleration,
                           const ros::Time &now, bool estimate_thrust,
                           mavros_msgs::AttitudeTarget &output,
                           quadrotor_msgs::Px4ctrlDebug &debug)
{
    if (!ready(now)) return false;
    Desired_State_t desired(impl_->odom);  // Reference jerk remains zero.
    desired.p = Eigen::Vector3d(target.position.x, target.position.y, target.position.z);
    desired.v = Eigen::Vector3d(velocity.x, velocity.y, velocity.z);
    desired.a = Eigen::Vector3d(acceleration.x, acceleration.y, acceleration.z);
    auto q = quaternion(target.orientation);
    if (!desired.p.allFinite() || !desired.v.allFinite() || !desired.a.allFinite() || !valid(q)) return false;
    desired.yaw = uav_utils::get_yaw_from_quaternion(q.normalized());
    if (estimate_thrust)
        impl_->controller->estimateThrustModel(impl_->imu.a_lp, param);
    Controller_Output_t control;
    debug = impl_->controller->calculateControl(desired, impl_->odom, impl_->imu, control);
    if (!valid(control.q) || !std::isfinite(control.thrust)) return false;
    // MAVROS normalized thrust must stay in [0,1]. No change to control formulas.
    setAttitude(output, control.q.normalized(), std::clamp(control.thrust, 0.0, 1.0), now);
    debug.header.stamp = now;
    return true;
}
