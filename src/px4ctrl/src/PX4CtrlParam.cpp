#include "PX4CtrlParam.h"

Parameter_t param;

Parameter_t::Parameter_t() {}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh) {
    read_essential_param(nh, "ctrl_mode", ctrl_mode);

    read_essential_param(nh, "gain/Kp0", gain.Kp0);
    read_essential_param(nh, "gain/Kp1", gain.Kp1);
    read_essential_param(nh, "gain/Kp2", gain.Kp2);
    read_essential_param(nh, "gain/Kv0", gain.Kv0);
    read_essential_param(nh, "gain/Kv1", gain.Kv1);
    read_essential_param(nh, "gain/Kv2", gain.Kv2);
    read_essential_param(nh, "gain/Kvi0", gain.Kvi0);
    read_essential_param(nh, "gain/Kvi1", gain.Kvi1);
    read_essential_param(nh, "gain/Kvi2", gain.Kvi2);
    read_essential_param(nh, "gain/KAngR", gain.KAngR);
    read_essential_param(nh, "gain/KAngP", gain.KAngP);
    read_essential_param(nh, "gain/KAngY", gain.KAngY);

    read_essential_param(nh, "mass", so3ctrl.m);
    read_essential_param(nh, "gra", so3ctrl.g);
    read_essential_param(nh, "gain/Kp0", so3ctrl.k_p[0]);
    read_essential_param(nh, "gain/Kp1", so3ctrl.k_p[1]);
    read_essential_param(nh, "gain/Kp2", so3ctrl.k_p[2]);
    read_essential_param(nh, "gain/Kv0", so3ctrl.k_v[0]);
    read_essential_param(nh, "gain/Kv1", so3ctrl.k_v[1]);
    read_essential_param(nh, "gain/Kv2", so3ctrl.k_v[2]);

    // MPC params are only required when using MPC mode.
    // (Many configs use ctrl_mode=0/1 and do not define mpc/* keys.)
    if (ctrl_mode == static_cast<int>(CtrlMode::mpc)) {
        // MPC horizon length is configured under the `mpc/` namespace in our YAMLs.
        // Keep a legacy fallback to root-level `N` to avoid breaking older configs.
        if (!nh.getParam("mpc/N", mpc_param.N)) {
            read_essential_param(nh, "N", mpc_param.N);
        }

        // Use global mass/gra unless you explicitly provide mpc/m and mpc/g elsewhere.
        mpc_param.m = so3ctrl.m;
        mpc_param.g = so3ctrl.g;

        read_essential_param(nh, "mpc/J0", mpc_param.J0);
        read_essential_param(nh, "mpc/J1", mpc_param.J1);
        read_essential_param(nh, "mpc/J2", mpc_param.J2);
        read_essential_param(nh, "mpc/Kp0", mpc_param.Kp0);
        read_essential_param(nh, "mpc/Kp1", mpc_param.Kp1);
        read_essential_param(nh, "mpc/Kp2", mpc_param.Kp2);
        read_essential_param(nh, "mpc/Kv0", mpc_param.Kv0);
        read_essential_param(nh, "mpc/Kv1", mpc_param.Kv1);
        read_essential_param(nh, "mpc/Kv2", mpc_param.Kv2);
        read_essential_param(nh, "mpc/KqR", mpc_param.KqR);
        read_essential_param(nh, "mpc/KqP", mpc_param.KqP);
        read_essential_param(nh, "mpc/KqY", mpc_param.KqY);
        read_essential_param(nh, "mpc/thrust_min", mpc_param.thrust_min);
        read_essential_param(nh, "mpc/thrust_max", mpc_param.thrust_max);
    }


    read_essential_param(nh, "rotor_drag/x", rt_drag.x);
    read_essential_param(nh, "rotor_drag/y", rt_drag.y);
    read_essential_param(nh, "rotor_drag/z", rt_drag.z);
    read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_thrust_horz);

    read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
    read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
    read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
    read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
    read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

    read_essential_param(nh, "pose_solver", pose_solver);
    read_essential_param(nh, "mass", mass);
    read_essential_param(nh, "gra", gra);
    read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
    read_essential_param(nh, "use_bodyrate_ctrl", use_bodyrate_ctrl);
    nh.param("use_bridge_forwarding", use_bridge_forwarding, false);
    read_essential_param(nh, "max_manual_vel", max_manual_vel);
    read_essential_param(nh, "max_angle", max_angle);
    read_essential_param(nh, "low_voltage", low_voltage);

    read_essential_param(nh, "imu_a_lpf_cutoff_freq", imu_a_lpf_cutoff_freq);

    read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
    read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
    read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
    read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

    read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
    read_essential_param(nh, "thrust_model/K1", thr_map.K1);
    read_essential_param(nh, "thrust_model/K2", thr_map.K2);
    read_essential_param(nh, "thrust_model/K3", thr_map.K3);
    read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
    read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);
    read_essential_param(nh, "thrust_model/rho2", thr_map.rho2);

    max_angle /= (180.0 / M_PI);

    if (thr_map.print_val) {
        ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
    }
};

// void Parameter_t::config_full_thrust(double hov)
// {
// 	full_thrust = mass * gra / hov;
// };
