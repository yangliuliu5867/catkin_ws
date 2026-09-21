#pragma once

#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <uav_utils/utils.h>

#include <Eigen/Dense>

#include "PX4CtrlParam.h"

// sample(当前的采样值) cutoff_freq(截止频率) dt(采样周期)
template<typename T>
void DigitalLPF(const T& sample, double cutoff_freq, double dt, T* const output) {
    T& _output = (*output);
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        _output = sample;
        return;
    }
    double rc = 1.0f / (2.0 * M_PI * cutoff_freq);
    double alpha = dt / (dt + rc);
    _output += (sample - _output) * alpha;
}

class RC_Data_t {
   public:
    double mode;
    double gear;
    double reboot_cmd;
    double last_mode;
    double last_gear;
    double last_reboot_cmd;
    bool have_init_last_mode{false};
    bool have_init_last_gear{false};
    bool have_init_last_reboot_cmd{false};
    double ch[4];

    mavros_msgs::RCIn msg;
    ros::Time rcv_stamp;

    bool is_command_mode;
    bool enter_command_mode;
    bool is_offboard_mode;
    bool enter_hover_mode;
    bool toggle_reboot;

    static constexpr int CH_MODE = (5 - 1);    // 切换offboard模式
    static constexpr int CH_REBOOT = (6 - 1);  // 飞控重启
    static constexpr int CH_GEAR = (9 - 1);    // 允许px4ctrl接收控制指令

    static constexpr double GEAR_SHIFT_VALUE = 0.75;
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;  // 大于此值表示进入offboard模式
    static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;
    static constexpr double DEAD_ZONE = 0.25;

    RC_Data_t();
    void check_validity();
    bool check_centered();
    void feed(mavros_msgs::RCInConstPtr pMsg);
    bool is_received(const ros::Time& now_time);
};

class Odom_Data_t {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;

    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    bool recv_new_msg;

    Odom_Data_t();
    void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Imu_Data_t {
   public:
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;

    Eigen::Vector3d a_lp;

    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;

    Imu_Data_t();
    void feed(sensor_msgs::ImuConstPtr pMsg);
};

class State_Data_t {
   public:
    mavros_msgs::State current_state;
    mavros_msgs::State state_before_offboard;

    State_Data_t();
    void feed(mavros_msgs::StateConstPtr pMsg);
};

class ExtendedState_Data_t {
   public:
    mavros_msgs::ExtendedState current_extended_state;

    ExtendedState_Data_t();
    void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};

class Command_Data_t {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    double yaw;
    double yaw_rate;

    quadrotor_msgs::PositionCommand msg;
    ros::Time rcv_stamp;

    Command_Data_t();
    void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

class BridgeCommand_Data_t {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double thrust_force;
    Eigen::Quaterniond quat;
    Eigen::Vector3d bodyrates;

    ros::Time thrust_rcv_stamp;
    ros::Time quat_rcv_stamp;
    ros::Time bodyrate_rcv_stamp;

    BridgeCommand_Data_t();
    void feedThrust(std_msgs::Float64ConstPtr pMsg);
    void feedQuat(geometry_msgs::QuaternionStampedConstPtr pMsg);
    void feedBodyrate(geometry_msgs::Vector3StampedConstPtr pMsg);

    bool thrust_is_received(const ros::Time &now_time, const double timeout) const;
    bool quat_is_received(const ros::Time &now_time, const double timeout) const;
    bool bodyrate_is_received(const ros::Time &now_time, const double timeout) const;
};

class Battery_Data_t {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double volt{0.0};
    double percentage{0.0};

    sensor_msgs::BatteryState msg;
    ros::Time rcv_stamp;

    Battery_Data_t();
    void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};

class Takeoff_Land_Data_t {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool triggered{false};
    uint8_t takeoff_land_cmd;  // see TakeoffLand.msg for its defination

    quadrotor_msgs::TakeoffLand msg;
    ros::Time rcv_stamp;

    Takeoff_Land_Data_t();
    void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};
