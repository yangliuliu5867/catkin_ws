#include "PX4CtrlFSM.h"

#include <algorithm>
#include <cmath>

#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

namespace {

double clampDouble(const double value, const double low, const double high) {
    return std::max(low, std::min(high, value));
}

}  // namespace

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_)
    : param(param_),
      controller(controller_) /*, thrust_curve(thrust_curve_)*/
{
    state = MANUAL_CTRL;
    hover_pose.setZero();
}

/*
        Finite State Machine

          system start
                |
                v
    ----- > MANUAL_CTRL 手动控制模式
    |         ^   |
    |         |   v
    |       AUTO_HOVER 遥控定点模式
    |         ^   |
    |         |   v
    -------- CMD_CTRL 指令控制模式

*/

void PX4CtrlFSM::process() {
    ros::Time now_time = ros::Time::now();
    Controller_Output_t u;
    Desired_State_t des(odom_data);

    // STEP1: state machine runs
    switch (state) {
        case MANUAL_CTRL: {
            // 跳转遥控定点模式
            if (rc_data.enter_hover_mode)  // Try to jump to AUTO_HOVER
            {
                const bool has_external_cmd = param.use_bridge_forwarding ? bridge_cmd_is_received(now_time) : cmd_is_received(now_time);
                if (!odom_is_received(now_time)) {
                    ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). No odom!");
                    break;
                }
                if (has_external_cmd) {
                    ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
                    break;
                }
                /*
                if (odom_data.v.norm() > 0.5) {
                    ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
                    break;
                } //*/

                state = AUTO_HOVER;
                controller.resetThrustMapping();
                set_hov_with_odom();
                toggle_offboard_mode(true);

                ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
            }

            if (rc_data.toggle_reboot)  // Try to reboot
            {
                if (state_data.current_state.armed) {
                    ROS_ERROR("[px4ctrl] Reject reboot! Disarm the drone first!");
                    break;
                }
                reboot_FCU();
            }

            break;
        }

        case AUTO_HOVER: {
            // 跳转手动控制模式
            if (!rc_data.is_offboard_mode || !odom_is_received(now_time)) {
                state = MANUAL_CTRL;
                toggle_offboard_mode(false);

                ROS_WARN("[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
            }
            // 跳转指令控制模式
            else if (rc_data.is_command_mode) {
                const bool has_external_cmd = param.use_bridge_forwarding ? bridge_cmd_is_received(now_time) : cmd_is_received(now_time);
                if (has_external_cmd) {
                    if (state_data.current_state.mode == "OFFBOARD") {
                        state = CMD_CTRL;
                        if (!param.use_bridge_forwarding) {
                            des = get_cmd_des();
                        }
                        ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
                    }
                } else {
                    publish_traj_trigger(odom_data.msg, true);
                }
            }
            // 进行遥控定点控制
            else {
                set_hov_with_rc();
                des = get_hover_des();
            }

            break;
        }

        case CMD_CTRL: {
            // 跳转手动控制模式
            if (!rc_data.is_offboard_mode || !odom_is_received(now_time)) {
                state = MANUAL_CTRL;
                toggle_offboard_mode(false);
                publish_traj_trigger(odom_data.msg, false);
                ROS_WARN("[px4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
            }
            // 跳转遥控定点控制
            else if (!rc_data.is_command_mode || !(param.use_bridge_forwarding ? bridge_cmd_is_received(now_time) : cmd_is_received(now_time))) {
                state = AUTO_HOVER;
                set_hov_with_odom();
                des = get_hover_des();
                publish_traj_trigger(odom_data.msg, false);
                ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
            }
            // 进行指令控制模式
            else {
                if (!param.use_bridge_forwarding) {
                    des = get_cmd_des();
                }
            }

            break;
        }

        default:
            break;
    }

    const bool forward_bridge_cmd = (state == CMD_CTRL) && param.use_bridge_forwarding && bridge_cmd_is_received(now_time);
    if (forward_bridge_cmd) {
        publish_bridge_forward_ctrl(now_time);
    } else {
        // STEP2: estimate thrust model
        if (state == AUTO_HOVER || state == CMD_CTRL) {
            // controller.estimateThrustModel(imu_data.a, bat_data.volt, param);
            controller.estimateThrustModel(imu_data.a_lp, param);
        }

        // STEP3: solve and update new control commands
        debug_msg = controller.calculateControl(des, odom_data, imu_data, u);
        debug_msg.header.stamp = now_time;
        debug_pub.publish(debug_msg);

        // STEP4: publish control commands to mavros
        if (param.use_bodyrate_ctrl) {
            publish_bodyrate_ctrl(u, now_time);
        } else {
            publish_attitude_ctrl(u, now_time);
        }
    }

    // STEP5: Clear flags beyound their lifetime
    rc_data.enter_hover_mode = false;
    rc_data.enter_command_mode = false;
    rc_data.toggle_reboot = false;
}

/*
void PX4CtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u) {
    u.q = imu.q;
    u.bodyrates = Eigen::Vector3d::Zero();
    u.thrust = 0.04;
} //*/

Desired_State_t PX4CtrlFSM::get_hover_des() {
    Desired_State_t des;
    des.p = hover_pose.head<3>();
    des.v = Eigen::Vector3d::Zero();
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = hover_pose(3);
    des.yaw_rate = 0.0;

    return des;
}

Desired_State_t PX4CtrlFSM::get_cmd_des() {
    Desired_State_t des;
    des.p = cmd_data.p;
    des.v = cmd_data.v;
    des.a = cmd_data.a;
    des.j = cmd_data.j;
    des.yaw = cmd_data.yaw;
    des.yaw_rate = cmd_data.yaw_rate;

    return des;
}

void PX4CtrlFSM::set_hov_with_odom() {
    hover_pose.head<3>() = odom_data.p;
    hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

    last_set_hover_pose_time = ros::Time::now();
}

void PX4CtrlFSM::set_hov_with_rc() {
    ros::Time now = ros::Time::now();
    double delta_t = (now - last_set_hover_pose_time).toSec();
    last_set_hover_pose_time = now;

    hover_pose(0) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1 : -1);
    hover_pose(1) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);
    hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? 1 : -1);
    hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);

    if (hover_pose(2) < -0.3) hover_pose(2) = -0.3;
}

bool PX4CtrlFSM::rc_is_received(const ros::Time &now_time) { return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc; }

bool PX4CtrlFSM::cmd_is_received(const ros::Time &now_time) { return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd; }

bool PX4CtrlFSM::bridge_cmd_is_received(const ros::Time &now_time) {
    const bool has_thrust = bridge_cmd_data.thrust_is_received(now_time, param.msg_timeout.cmd);
    if (!has_thrust) {
        return false;
    }

    if (param.use_bodyrate_ctrl) {
        return bridge_cmd_data.bodyrate_is_received(now_time, param.msg_timeout.cmd);
    }
    return bridge_cmd_data.quat_is_received(now_time, param.msg_timeout.cmd);
}

bool PX4CtrlFSM::odom_is_received(const ros::Time &now_time) { return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom; }

bool PX4CtrlFSM::imu_is_received(const ros::Time &now_time) { return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu; }

bool PX4CtrlFSM::bat_is_received(const ros::Time &now_time) { return (now_time - bat_data.rcv_stamp).toSec() < param.msg_timeout.bat; }

bool PX4CtrlFSM::recv_new_odom() {
    if (odom_data.recv_new_msg) {
        odom_data.recv_new_msg = false;
        return true;
    }

    return false;
}

void PX4CtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp) {
    mavros_msgs::AttitudeTarget msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = std::string("FCU");

    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

    msg.body_rate.x = u.bodyrates.x();
    msg.body_rate.y = u.bodyrates.y();
    msg.body_rate.z = u.bodyrates.z();

    msg.thrust = u.thrust;

    ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp) {
    mavros_msgs::AttitudeTarget msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = std::string("FCU");

    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    msg.orientation.x = u.q.x();
    msg.orientation.y = u.q.y();
    msg.orientation.z = u.q.z();
    msg.orientation.w = u.q.w();

    msg.thrust = u.thrust;

    ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_bridge_forward_ctrl(const ros::Time &stamp) {
    const double mass = std::max(1e-3, param.mass);
    const double thr2acc = std::max(1e-3, param.thr_map.thr2acc);

    double throttle = bridge_cmd_data.thrust_force / mass / thr2acc;
    if (!std::isfinite(throttle)) {
        throttle = 0.0;
    }
    throttle = clampDouble(throttle, 0.0, 1.0);

    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = std::string("FCU");
    msg.thrust = throttle;

    if (param.use_bodyrate_ctrl) {
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        msg.body_rate.x = bridge_cmd_data.bodyrates.x();
        msg.body_rate.y = bridge_cmd_data.bodyrates.y();
        msg.body_rate.z = bridge_cmd_data.bodyrates.z();
    } else {
        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        msg.orientation.x = bridge_cmd_data.quat.x();
        msg.orientation.y = bridge_cmd_data.quat.y();
        msg.orientation.z = bridge_cmd_data.quat.z();
        msg.orientation.w = bridge_cmd_data.quat.w();
    }
    ctrl_FCU_pub.publish(msg);

    debug_msg.header.stamp = stamp;
    debug_msg.des_q_x = bridge_cmd_data.quat.x();
    debug_msg.des_q_y = bridge_cmd_data.quat.y();
    debug_msg.des_q_z = bridge_cmd_data.quat.z();
    debug_msg.des_q_w = bridge_cmd_data.quat.w();
    debug_msg.des_thr = throttle;
    debug_pub.publish(debug_msg);
}

void PX4CtrlFSM::publish_traj_trigger(const nav_msgs::Odometry &odom_msg, const bool on_off) {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = on_off ? "start" : "stop";
    msg.pose = odom_msg.pose.pose;

    traj_start_trigger_pub.publish(msg);
}

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off) {
    mavros_msgs::SetMode offb_set_mode;

    if (on_off) {
        state_data.state_before_offboard = state_data.current_state;
        if (state_data.state_before_offboard.mode == "OFFBOARD")  // Not allowed
            state_data.state_before_offboard.mode = "MANUAL";

        offb_set_mode.request.custom_mode = "OFFBOARD";
        if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent)) {
            ROS_ERROR("Enter OFFBOARD rejected by PX4!");
            return false;
        }
    } else {
        offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
        if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent)) {
            ROS_ERROR("Exit OFFBOARD rejected by PX4!");
            return false;
        }
    }

    return true;
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;
    if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success)) {
        if (arm)
            ROS_ERROR("ARM rejected by PX4!");
        else
            ROS_ERROR("DISARM rejected by PX4!");

        return false;
    }

    return true;
}

void PX4CtrlFSM::reboot_FCU() {
    // https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
    mavros_msgs::CommandLong reboot_srv;
    reboot_srv.request.broadcast = false;
    reboot_srv.request.command = 246;  // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    reboot_srv.request.param1 = 1;     // Reboot autopilot
    reboot_srv.request.param2 = 0;     // Do nothing for onboard computer
    reboot_srv.request.confirmation = true;

    reboot_FCU_srv.call(reboot_srv);

    ROS_INFO("Reboot FCU");
}
