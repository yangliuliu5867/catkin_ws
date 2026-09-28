#ifndef FSM_CTRL_SINGLE_OFFBOARD_FSM_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_FSM_HPP_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <string>
#include <vector>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/PositionCommandArray.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <fsm_ctrl/NMPC_Controller.hpp>
#include <fsm_ctrl/nmpc_state.h>

#endif  // FSM_CTRL_SINGLE_OFFBOARD_FSM_HPP_
