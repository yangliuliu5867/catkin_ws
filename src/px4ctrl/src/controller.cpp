#include "controller.h"

using namespace std;


double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q) {
    double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
}

LinearControl::LinearControl(Parameter_t& param) : param_(param) { resetThrustMapping(); }

/*
  compute u.thrust and u.q, controller gains and other parameters are in param_
*/
quadrotor_msgs::Px4ctrlDebug LinearControl::calculateMPCControl(const Desired_State_t& des, const Odom_Data_t& odom, const Imu_Data_t& imu, Controller_Output_t& u) {
    // TODO: 添加 MPC 算法的实际实现。
    // 目前作为占位示例，仅简单沿用des中的加速度和角度。
    Eigen::Vector3d des_acc = des.a;  
    Eigen::Quaterniond des_q = des.q;  

    // 计算推力，注意：computeDesiredCollectiveThrustSignal基于已有建模
    u.thrust = computeDesiredCollectiveThrustSignal(odom.q.conjugate() * des_acc);
    
    // 用 imu 与 odom 的信息构造期望姿态
    u.q = imu.q * odom.q.conjugate() * des_q;
    if (u.q.w() * imu.q.w() < 0) {
        u.q.coeffs() = -u.q.coeffs();
    }
    
    // 可在 debug_msg_ 中填入调试信息（此处简单赋值）
    debug_msg_.des_p_x = des.p(0);
    debug_msg_.des_p_y = des.p(1);
    debug_msg_.des_p_z = des.p(2);
    debug_msg_.des_a_x = des_acc(0);
    debug_msg_.des_a_y = des_acc(1);
    debug_msg_.des_a_z = des_acc(2);
    debug_msg_.des_q_x = u.q.x();
    debug_msg_.des_q_y = u.q.y();
    debug_msg_.des_q_z = u.q.z();
    debug_msg_.des_q_w = u.q.w();
    debug_msg_.des_thr = u.thrust;

    return debug_msg_;
}

quadrotor_msgs::Px4ctrlDebug LinearControl::calculateControl(const Desired_State_t& des, const Odom_Data_t& odom, const Imu_Data_t& imu, Controller_Output_t& u) {
    Eigen::Vector3d des_acc;
    Eigen::Quaterniond des_q;

    if (param_.ctrl_mode == Parameter_t::CtrlMode::so3) {
        Eigen::Vector3d F_world_sp;
        Eigen::Matrix3d R_sp;
        Eigen::Vector3d e_p_world;
        Eigen::Vector3d e_v_world;
        OuterLoopControl(odom.p,  //
                         odom.v,
                         des.p,
                         des.v,
                         des.a,
                         des.yaw,
                         param_.so3ctrl,
                         1e-5,
                         &F_world_sp,
                         &R_sp,
                         &e_p_world,
                         &e_v_world);
        des_acc = F_world_sp / param_.mass + Eigen::Vector3d(0, 0, param_.gra);
        des_q = Eigen::Quaterniond(R_sp);
    } else {
        Eigen::Vector3d Kp(param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2);
        Eigen::Vector3d Kv(param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2);
        Eigen::Vector3d Kvi(param_.gain.Kvi0, param_.gain.Kvi1, param_.gain.Kvi2);
        Eigen::Vector3d Kvd(param_.gain.Kvd0, param_.gain.Kvd1, param_.gain.Kvd2);

        Eigen::Vector3d Ep = des.p - odom.p;
        Eigen::Vector3d Ev = des.v - odom.v;

        static ros::Time lastT = ros::Time::now();
        double deltaT = (ros::Time::now() - lastT).toSec();
        lastT = ros::Time::now();

        static Eigen::Vector3d lastEv = Ev;
        Eigen::Vector3d Evd = Ev - lastEv;
        if (deltaT > 0.005 && deltaT < 0.1) {
            Evd /= deltaT;
        } else {
            Evd.setZero();
        }
        lastEv = Ev;

        static Eigen::Vector3d Evi = Eigen::Vector3d::Zero();
        if (deltaT > 0.005 && deltaT < 0.1) {
            Evi += Ev * deltaT;
        } else {
            Evi.setZero();
        }

        des_acc = des.a + Kv.asDiagonal() * Ev + Kvi.asDiagonal() * Evi + Kvd.asDiagonal() * Evd + Kp.asDiagonal() * Ep;
        des_acc += Eigen::Vector3d(0, 0, param_.gra);

        double roll, pitch;
        double yaw_odom = fromQuaternion2yaw(odom.q);
        double sin = std::sin(yaw_odom);
        double cos = std::cos(yaw_odom);
        roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra;
        pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;

        des_q = Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    }

    // des_acc is in world frame, not body frame
    u.thrust = computeDesiredCollectiveThrustSignal(odom.q.conjugate() * des_acc);

    // Eigen::Quaterniond q_align(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    // Eigen::Quaterniond odom_aligned = q_align * odom.q;
    u.q = imu.q * odom.q.conjugate() * des_q;
    if (u.q.w() * imu.q.w() < 0) {
        u.q.w() = -u.q.w();
        u.q.x() = -u.q.x();
        u.q.y() = -u.q.y();
        u.q.z() = -u.q.z();
    }

    // used for debug
    debug_msg_.des_p_x = des.p(0);
    debug_msg_.des_p_y = des.p(1);
    debug_msg_.des_p_z = des.p(2);

    debug_msg_.des_v_x = des.v(0);
    debug_msg_.des_v_y = des.v(1);
    debug_msg_.des_v_z = des.v(2);

    debug_msg_.des_a_x = des_acc(0);
    debug_msg_.des_a_y = des_acc(1);
    debug_msg_.des_a_z = des_acc(2);

    debug_msg_.des_q_x = u.q.x();
    debug_msg_.des_q_y = u.q.y();
    debug_msg_.des_q_z = u.q.z();
    debug_msg_.des_q_w = u.q.w();

    debug_msg_.des_thr = u.thrust;

    // Used for thrust-accel mapping estimation
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrust_.size() > 100) {
        timed_thrust_.pop();
    }
    return debug_msg_;
}

/*
  compute throttle percentage
*/
double LinearControl::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d& des_acc) {
    double throttle_percentage(0.0);

    /* compute throttle, thr2acc has been estimated before */
    throttle_percentage = des_acc(2) / param_.thr_map.thr2acc;
    throttle_percentage = throttle_percentage < 1.0 ? throttle_percentage : 1.0;
    throttle_percentage = throttle_percentage > -1.0 ? throttle_percentage : -1.0;

    return throttle_percentage;
}

bool LinearControl::estimateThrustModel(const Eigen::Vector3d& est_a, const Parameter_t& param) {
    ros::Time t_now = ros::Time::now();
    while (timed_thrust_.size() >= 1) {
        // Choose data before 35~45ms ago
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        double time_passed = (t_now - t_t.first).toSec();
        if (time_passed > 0.045)  // 45ms
        {
            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            continue;
        }
        if (time_passed < 0.035)  // 35ms
        {
            // printf("skip, time_passed=%f\n", time_passed);
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thr = t_t.second;
        timed_thrust_.pop();

        /***********************************/
        /* Model: est_a(2) = thr1acc_ * thr */
        /***********************************/
        double gamma = 1 / (param_.thr_map.rho2 + thr * param_.thr_map.P_thr2acc * thr);
        double K = gamma * param_.thr_map.P_thr2acc * thr;
        param_.thr_map.thr2acc = param_.thr_map.thr2acc + K * (est_a(2) - thr * param_.thr_map.thr2acc);
        param_.thr_map.P_thr2acc = (1 - K * thr) * param_.thr_map.P_thr2acc / param_.thr_map.rho2;

        debug_msg_.thr2acc = param_.thr_map.thr2acc;
        debug_msg_.thr2acc_P = param_.thr_map.P_thr2acc;

        return true;
    }
    return false;
}

void LinearControl::resetThrustMapping(void) {
    param_.thr_map.thr2acc = param_.gra / param_.thr_map.hover_percentage;
    param_.thr_map.P_thr2acc = 1e6;
    debug_msg_.thr2acc = param_.thr_map.thr2acc;
    debug_msg_.thr2acc_P = param_.thr_map.P_thr2acc;
}

void LinearControl::OuterLoopControl(const Eigen::Matrix<double, 3, 1>& p_world, const Eigen::Matrix<double, 3, 1>& v_world, const Eigen::Matrix<double, 3, 1>& p_world_sp, const Eigen::Matrix<double, 3, 1>& v_world_sp, const Eigen::Matrix<double, 3, 1>& a_world_sp, const double yaw_sp, const Parameter_t::SO3Ctrl& params, const double epsilon, Eigen::Matrix<double, 3, 1>* const F_world_sp, Eigen::Matrix<double, 3, 3>* const R_sp, Eigen::Matrix<double, 3, 1>* const e_p_world, Eigen::Matrix<double, 3, 1>* const e_v_world) {
    // Total ops: 86

    // Input arrays

    // Intermediate terms (30)
    const double _tmp0 = -p_world(0, 0) + p_world_sp(0, 0);
    const double _tmp1 = -v_world(0, 0) + v_world_sp(0, 0);
    const double _tmp2 = _tmp0 * params.k_p[0] * params.m + _tmp1 * params.k_v[0] * params.m + a_world_sp(0, 0) * params.m;
    const double _tmp3 = -p_world(1, 0) + p_world_sp(1, 0);
    const double _tmp4 = -v_world(1, 0) + v_world_sp(1, 0);
    const double _tmp5 = _tmp3 * params.k_p[1] * params.m + _tmp4 * params.k_v[1] * params.m + a_world_sp(1, 0) * params.m;
    const double _tmp6 = -p_world(2, 0) + p_world_sp(2, 0);
    const double _tmp7 = -v_world(2, 0) + v_world_sp(2, 0);
    const double _tmp8 = _tmp6 * params.k_p[2] * params.m + _tmp7 * params.k_v[2] * params.m + a_world_sp(2, 0) * params.m - params.g * params.m;
    const double _tmp9 = std::cos(yaw_sp);
    const double _tmp10 = std::pow(_tmp2, double(2)) + std::pow(_tmp5, double(2)) + std::pow(_tmp8, double(2));
    const double _tmp11 = (((std::sqrt(_tmp10)) > 0) - ((std::sqrt(_tmp10)) < 0));
    const double _tmp12 = std::pow(double(_tmp10 + epsilon * (1 - _tmp11)), double(double(-1) / double(2)));
    const double _tmp13 = _tmp11 * (_tmp12 * _tmp8 - 1) + 1;
    const double _tmp14 = std::pow(_tmp13, double(2));
    const double _tmp15 = std::sin(yaw_sp);
    const double _tmp16 = _tmp11 * _tmp12;
    const double _tmp17 = _tmp16 * _tmp2;
    const double _tmp18 = _tmp16 * _tmp5;
    const double _tmp19 = _tmp15 * _tmp17 - _tmp18 * _tmp9;
    const double _tmp20 = std::pow(double(_tmp14 * std::pow(_tmp15, double(2)) + _tmp14 * std::pow(_tmp9, double(2)) + std::pow(_tmp19, double(2))), double(double(-1) / double(2)));
    const double _tmp21 = _tmp14 * _tmp20;
    const double _tmp22 = _tmp19 * _tmp20;
    const double _tmp23 = -_tmp18 * _tmp22 + _tmp21 * _tmp9;
    const double _tmp24 = _tmp15 * _tmp21 + _tmp17 * _tmp22;
    const double _tmp25 = _tmp13 * _tmp20;
    const double _tmp26 = _tmp25 * _tmp9;
    const double _tmp27 = _tmp15 * _tmp25;
    const double _tmp28 = -_tmp17 * _tmp26 - _tmp18 * _tmp27;
    const double _tmp29 = std::pow(double(std::pow(_tmp23, double(2)) + std::pow(_tmp24, double(2)) + std::pow(_tmp28, double(2))), double(double(-1) / double(2)));

    // Output terms (4)
    if (F_world_sp != nullptr) {
        Eigen::Matrix<double, 3, 1>& _F_world_sp = (*F_world_sp);


        _F_world_sp(0, 0) = _tmp2;
        _F_world_sp(1, 0) = _tmp5;
        _F_world_sp(2, 0) = _tmp8;
    }

    if (R_sp != nullptr) {
        Eigen::Matrix<double, 3, 3>& _R_sp = (*R_sp);


        _R_sp(0, 0) = _tmp23 * _tmp29;
        _R_sp(1, 0) = _tmp24 * _tmp29;
        _R_sp(2, 0) = _tmp28 * _tmp29;
        _R_sp(0, 1) = -_tmp27;
        _R_sp(1, 1) = _tmp26;
        _R_sp(2, 1) = _tmp22;
        _R_sp(0, 2) = _tmp17;
        _R_sp(1, 2) = _tmp18;
        _R_sp(2, 2) = _tmp13;
    }

    if (e_p_world != nullptr) {
        Eigen::Matrix<double, 3, 1>& _e_p_world = (*e_p_world);


        _e_p_world(0, 0) = _tmp0;
        _e_p_world(1, 0) = _tmp3;
        _e_p_world(2, 0) = _tmp6;
    }

    if (e_v_world != nullptr) {
        Eigen::Matrix<double, 3, 1>& _e_v_world = (*e_v_world);


        _e_v_world(0, 0) = _tmp1;
        _e_v_world(1, 0) = _tmp4;
        _e_v_world(2, 0) = _tmp7;
    }
}  // NOLINT(readability/fn_size)
