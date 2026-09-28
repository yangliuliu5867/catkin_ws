#ifndef _PX4_ESTIMATOR_HPP_
#define _PX4_ESTIMATOR_HPP_

#include <cmath>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <ctrl_math/ctrl_math.hpp>

static Eigen::Vector3d pos_fcu = Eigen::Vector3d::Zero();
static Eigen::Quaterniond quat_fcu = Eigen::Quaterniond::Identity();
static Eigen::Vector3d euler_fcu = Eigen::Vector3d::Zero();
static double lidar_to_body_pitch_deg = 0.0;

static std_msgs::Bool ekf_ready;
static geometry_msgs::PoseStamped vision_pose;
static ros::Publisher vision_pub;
static ros::Publisher selected_odom_pub;

void LidarCallback(const nav_msgs::Odometry::ConstPtr &msg);
void FcuPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

#endif
