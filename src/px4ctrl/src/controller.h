/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#pragma once

#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>

#include <Eigen/Dense>
#include <queue>

#include "input.h"

struct Desired_State_t {
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Quaterniond q;
    double yaw;
    double yaw_rate;

    Desired_State_t(){};

    Desired_State_t(Odom_Data_t& odom) : p(odom.p), v(Eigen::Vector3d::Zero()), a(Eigen::Vector3d::Zero()), j(Eigen::Vector3d::Zero()), q(odom.q), yaw(uav_utils::get_yaw_from_quaternion(odom.q)), yaw_rate(0){};
};

struct Controller_Output_t {
    // Orientation of the body frame with respect to the world frame
    Eigen::Quaterniond q;

    // Body rates in body frame
    Eigen::Vector3d bodyrates;  // [rad/s]

    // Collective mass normalized thrust
    double thrust;

    // Eigen::Vector3d des_v_real;
};


class LinearControl {
   public:
    LinearControl(Parameter_t&);
    quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t& des, const Odom_Data_t& odom, const Imu_Data_t& imu, Controller_Output_t& u);
    quadrotor_msgs::Px4ctrlDebug calculateMPCControl(const Desired_State_t& des, const Odom_Data_t& odom, const Imu_Data_t& imu, Controller_Output_t& u);
    bool estimateThrustModel(const Eigen::Vector3d& est_v, const Parameter_t& param);
    void resetThrustMapping(void);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   private:
    Parameter_t param_;
    quadrotor_msgs::Px4ctrlDebug debug_msg_;
    std::queue<std::pair<ros::Time, double>> timed_thrust_;

    double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d& des_acc);
    double fromQuaternion2yaw(Eigen::Quaterniond q);
    void OuterLoopControl(const Eigen::Matrix<double, 3, 1>& p_world, const Eigen::Matrix<double, 3, 1>& v_world, const Eigen::Matrix<double, 3, 1>& p_world_sp, const Eigen::Matrix<double, 3, 1>& v_world_sp, const Eigen::Matrix<double, 3, 1>& a_world_sp, const double yaw_sp, const Parameter_t::SO3Ctrl& params, const double epsilon, Eigen::Matrix<double, 3, 1>* const F_world_sp = nullptr, Eigen::Matrix<double, 3, 3>* const R_sp = nullptr, Eigen::Matrix<double, 3, 1>* const e_p_world = nullptr, Eigen::Matrix<double, 3, 1>* const e_v_world = nullptr);
};
