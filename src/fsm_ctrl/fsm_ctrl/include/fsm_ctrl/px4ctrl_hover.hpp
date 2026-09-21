#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <memory>

// Thin data adapter. Control and thrust estimation remain in original px4ctrl.
class Px4CtrlHover
{
public:
    explicit Px4CtrlHover(const ros::NodeHandle &params);
    ~Px4CtrlHover();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    bool ready(const ros::Time &now) const;
    void reset();
    bool calculate(const geometry_msgs::Pose &target, const ros::Time &now,
                   bool estimate_thrust, mavros_msgs::AttitudeTarget &output,
                   quadrotor_msgs::Px4ctrlDebug &debug);
    bool calculate(const geometry_msgs::Pose &target,
                   const geometry_msgs::Vector3 &velocity,
                   const geometry_msgs::Vector3 &acceleration,
                   const ros::Time &now, bool estimate_thrust,
                   mavros_msgs::AttitudeTarget &output,
                   quadrotor_msgs::Px4ctrlDebug &debug);
    mavros_msgs::AttitudeTarget idle(const ros::Time &now) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
