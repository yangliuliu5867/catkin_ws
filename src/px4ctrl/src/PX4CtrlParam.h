#pragma once

#include <ros/ros.h>

class Parameter_t {
   public:
    enum CtrlMode {
        origin = 0,
        so3 = 1,
        mpc = 2,
    };

    struct Gain {
        double Kp0, Kp1, Kp2;
        double Kv0, Kv1, Kv2;
        double Kvi0, Kvi1, Kvi2;
        double Kvd0, Kvd1, Kvd2;
        double KAngR, KAngP, KAngY;
    };

    struct SO3Ctrl {
        double m;
        double g;
        std::array<double, 3> k_p;
        std::array<double, 3> k_v;
    };

    struct MPC {
        double N;
        double m;
        double g;
        double J0, J1, J2;
        double Kp0, Kp1, Kp2;
        double Kv0, Kv1, Kv2;
        double KqR, KqP, KqY;
        double thrust_min;
        double thrust_max;
    };

    struct RotorDrag {
        double x, y, z;
        double k_thrust_horz;
    };

    struct MsgTimeout {
        double odom;
        double rc;
        double cmd;
        double imu;
        double bat;
    };

    struct ThrustMapping {
        bool print_val;
        double K1;
        double K2;
        double K3;
        bool accurate_thrust_model;
        double hover_percentage;
        // Thrust-accel mapping params
        double rho2 = 0.998;
        double thr2acc;    // autoset in resetThrustMapping
        double P_thr2acc;  // autoset in resetThrustMapping
    };

    struct RCReverse {
        bool roll;
        bool pitch;
        bool yaw;
        bool throttle;
    };

    Gain gain;
    SO3Ctrl so3ctrl;
    MPC mpc_param;
    RotorDrag rt_drag;
    MsgTimeout msg_timeout;
    RCReverse rc_reverse;
    ThrustMapping thr_map;

    int pose_solver;
    double mass;
    double gra;
    double max_angle;
    double ctrl_freq_max;
    double max_manual_vel;
    double low_voltage;
    double imu_a_lpf_cutoff_freq;

    int ctrl_mode;

    bool use_bodyrate_ctrl;
    bool use_bridge_forwarding;
    // bool print_dbg;

    Parameter_t();

    void config_from_ros_handle(const ros::NodeHandle &nh);

   private:
    template<typename TName, typename TVal>
    void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val) {
        if (nh.getParam(name, val)) {
            // pass
        } else {
            ROS_ERROR_STREAM("Read param: " << name << " failed.");
            ROS_BREAK();
        }
    };
};

extern Parameter_t param;
