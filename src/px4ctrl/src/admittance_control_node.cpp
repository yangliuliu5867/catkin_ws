#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <uav_utils/utils.h>

class AdmittanceControl {
   private:
    Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d M_inv = Eigen::Matrix3d::Identity();
    // std::vector<double> segDurations;
    // std::vector<Eigen::MatrixXd> polyCoeffs;

   private:
    bool init_rf;
    Eigen::Vector3d p_world_rf;
    Eigen::Vector3d v_world_rf;
    Eigen::Vector3d a_world_rf;
    double traj_start_time;

    double delta_t;

   public:
    AdmittanceControl(ros::NodeHandle& nh, double dt) {
        getParam(nh, "AdmittanceControl/M_vx", M.coeffRef(0, 0));
        getParam(nh, "AdmittanceControl/M_vy", M.coeffRef(1, 1));
        getParam(nh, "AdmittanceControl/M_vz", M.coeffRef(2, 2));
        getParam(nh, "AdmittanceControl/D_vx", D.coeffRef(0, 0));
        getParam(nh, "AdmittanceControl/D_vy", D.coeffRef(1, 1));
        getParam(nh, "AdmittanceControl/D_vz", D.coeffRef(2, 2));
        getParam(nh, "AdmittanceControl/K_vx", K.coeffRef(0, 0));
        getParam(nh, "AdmittanceControl/K_vy", K.coeffRef(1, 1));
        getParam(nh, "AdmittanceControl/K_vz", K.coeffRef(2, 2));
        // // 获取分段持续时间
        // if (!nh.getParam("AdmittanceControl/segDurations", segDurations)) {
        //     ROS_ERROR_STREAM("Failed to get AdmittanceControl/segDurations");
        //     ROS_BREAK();
        // }
        
        // // 读取多段多项式系数
        // XmlRpc::XmlRpcValue coeffs_list;
        // if (!nh.getParam("AdmittanceControl/polyCoeffs", coeffs_list)) {
        //     ROS_ERROR_STREAM("Failed to get AdmittanceControl/polyCoeffs");
        //     ROS_BREAK();
        // }
        // // 期望有3段，每段 8×3
        // if (coeffs_list.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        //     coeffs_list.size() != 3) {
        //     ROS_ERROR_STREAM("polyCoeffs should be an array of 3 segments.");
        //     ROS_BREAK();
        // }
        // for (int i = 0; i < coeffs_list.size(); i++) {
        //     // 每段应为8行
        //     if (coeffs_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray ||
        //         int(coeffs_list[i].size()) != 8) {
        //         ROS_ERROR_STREAM("Each segment in polyCoeffs must have 8 rows.");
        //         ROS_BREAK();
        //     }
        //     Eigen::MatrixXd coeff(8, 3);
        //     for (int r = 0; r < 8; r++) {
        //         if (coeffs_list[i][r].getType() != XmlRpc::XmlRpcValue::TypeArray ||
        //             int(coeffs_list[i][r].size()) != 3) {
        //             ROS_ERROR_STREAM("Each row in polyCoeffs segment must have 3 columns.");
        //             ROS_BREAK();
        //         }
        //         for (int c = 0; c < 3; c++) {
        //             coeff(r, c) = static_cast<double>(coeffs_list[i][r][c]);
        //         }
        //     }
        //     polyCoeffs.push_back(coeff);
        // }
        M_inv = M.inverse();

        initReference();
        setDeltaT(dt);
    }

    ~AdmittanceControl() {
        //
    }

    void initReference() {
        init_rf = true;
        traj_start_time = ros::Time::now().toSec(); // Initialize trajectory start time
        return;
    }

    void setDeltaT(double dt) {
        delta_t = dt;
        return;
    }

    // 评估分段多项式轨迹，此函数根据当前时间 t，确定使用哪一个轨迹段，并计算期望状态
    // void evaluateSegmentedTrajectory(double t, double traj_start_time,
    //                                 const std::vector<double>& segDurations,
    //                                 const std::vector<Eigen::MatrixXd>& polyCoeffs,
    //                                 Eigen::Vector3d& pos,
    //                                 Eigen::Vector3d& vel,
    //                                 Eigen::Vector3d& acc) {
    //     // 计算相对于 traj_start_time 的相对时间
    //     double t_rel = t - traj_start_time;
    //     // 找到当前所在的轨迹段
    //     int seg = 0;
    //     double cumTime = 0.0;
    //     for (size_t i = 0; i < segDurations.size(); i++) {
    //         if (t_rel < cumTime + segDurations[i]) {
    //             seg = i;
    //             break;
    //         }
    //         cumTime += segDurations[i];
    //     }
    //     // 剩余时间（在当前段的时间）
    //     double t_seg = t_rel - cumTime;
        
    //     // 取出当前轨迹段的多项式系数, 假设 polyCoeffs[seg] 的尺寸为 (n+1)×3
    //     const Eigen::MatrixXd& coeffs = polyCoeffs[seg];
    //     int order = coeffs.rows() - 1;
        
    //     pos.setZero();
    //     vel.setZero();
    //     acc.setZero();
        
    //     // 评估多项式
    //     for (int i = 0; i <= order; i++) {
    //         double t_pow = std::pow(t_seg, i);
    //         pos += coeffs.row(i).transpose() * t_pow;
    //         if (i >= 1) {
    //             vel += i * coeffs.row(i).transpose() * std::pow(t_seg, i - 1);
    //         }
    //         if (i >= 2) {
    //             acc += i * (i - 1) * coeffs.row(i).transpose() * std::pow(t_seg, i - 2);
    //         }
    //     }
    // }

    

    void Control(const Eigen::Vector3d F_ext, const Eigen::Vector3d& p_world, const Eigen::Vector3d& v_world, Eigen::Vector3d* const p_world_sp_ptr, Eigen::Vector3d* const v_world_sp_ptr, Eigen::Vector3d* const a_world_sp_ptr, double* const yaw_sp_ptr) {
        Eigen::Vector3d& p_world_sp = (*p_world_sp_ptr);
        Eigen::Vector3d& v_world_sp = (*v_world_sp_ptr);
        Eigen::Vector3d& a_world_sp = (*a_world_sp_ptr);
        double& yaw_sp = (*yaw_sp_ptr);

        //*
        a_world_sp.setZero();
        v_world_sp.setZero();
        p_world_sp += v_world_sp * delta_t + 0.5 * a_world_sp * delta_t * delta_t;
        //*/

        if (init_rf) {
            init_rf = false;
            p_world_sp = p_world_rf = p_world;
            v_world_sp = v_world_rf = v_world;
        }

        // // 用分段多项式计算期望状态
        // double t_now = ros::Time::now().toSec();
        // evaluateSegmentedTrajectory(t_now, traj_start_time, segDurations, polyCoeffs, p_world_sp, v_world_sp, a_world_sp);

        a_world_rf = M_inv * F_ext + a_world_sp;
        a_world_rf -= M_inv * D * (v_world_rf - v_world_sp);
        a_world_rf -= M_inv * K * (p_world_rf - p_world_sp);

        // 设定轨迹会重新撞向障碍物
        // p_world_rf = p_world + v_world_rf * delta_t + 0.5 * a_world_rf * delta_t * delta_t;
        // v_world_rf = v_world + a_world_rf * delta_t;

        p_world_rf += v_world_rf * delta_t + 0.5 * a_world_rf * delta_t * delta_t;
        v_world_rf += a_world_rf * delta_t;

        p_world_sp = p_world_rf;
        v_world_sp = v_world_rf;
        a_world_sp = a_world_rf;
        yaw_sp = 0.0;

           // 计算当前经过时间
        double t_elapsed = ros::Time::now().toSec() - traj_start_time;
        // 三段式参数（加速、匀速、减速）
        double t_acc = 3.0;      // 加速阶段
        double t_const = 2.33;   // 匀速阶段
        double t_total = 8.33;   // 总时长

        double x = 0.0, vx = 0.0, ax = 0.0;
        if (t_elapsed < t_acc) {
            // 加速阶段，a = 0.5 m/s²
            ax = 0.5;
            vx = ax * t_elapsed;
            x = 0.5 * ax * t_elapsed * t_elapsed;
        } else if (t_elapsed < t_acc + t_const) {
            // 匀速阶段，速度保持 1.5 m/s
            ax = 0.0;
            vx = 1.5;
            // 加速阶段运动位移为 0.5*0.5*3² = 2.25m
            x = 2.25 + 1.5 * (t_elapsed - t_acc);
        } else if (t_elapsed <= t_total) {
            // 减速阶段，a = -0.5 m/s²
            double t_dec = t_elapsed - t_acc - t_const;
            ax = -0.5;
            vx = 1.5 + ax * t_dec; // 1.5 - 0.5*t_dec
            // 位移：加速阶段+匀速阶段位移为 2.25 + 1.5*t_const，
            // 减速阶段位移：1.5*t_dec + 0.5*ax*t_dec² (注意加速度为负)
            x = 2.25 + 1.5 * t_const + 1.5 * t_dec + 0.5 * ax * t_dec * t_dec;
        } else {
            // 运动结束后保持终点状态
            ax = 0.0;
            vx = 0.0;
            x = 8.0;
        }

        // 仅沿 x 轴规划轨迹，其余轴保持初始状态
        p_world_sp = Eigen::Vector3d(x, p_world_sp.y(), p_world_sp.z());
        v_world_sp = Eigen::Vector3d(vx, v_world_sp.y(), v_world_sp.z());
        a_world_sp = Eigen::Vector3d(ax, a_world_sp.y(), a_world_sp.z());
        yaw_sp = 0.0;

        return;
    }

    template<typename TName, typename TVal>
    void getParam(const ros::NodeHandle& nh, const TName& name, TVal& val) {
        if (nh.getParam(name, val) == false) {
            ROS_ERROR_STREAM("Read param: " << name << " failed.");
            ROS_BREAK();
        }
    };
};

Eigen::Vector3d point3MsgToEigen(const geometry_msgs::Point& msg) { return Eigen::Vector3d(msg.x, msg.y, msg.z); }

geometry_msgs::Point eigenToPoint3Msg(const Eigen::Vector3d& vec) {
    geometry_msgs::Point vec3;
    vec3.x = vec.x();
    vec3.y = vec.y();
    vec3.z = vec.z();
    return vec3;
}

Eigen::Vector3d vector3MsgToEigen(const geometry_msgs::Vector3& msg) { return Eigen::Vector3d(msg.x, msg.y, msg.z); }

geometry_msgs::Vector3 eigenToVector3Msg(const Eigen::Vector3d& vec) {
    geometry_msgs::Vector3 vec3;
    vec3.x = vec.x();
    vec3.y = vec.y();
    vec3.z = vec.z();
    return vec3;
}

double remove_noise(double val, double thr = 1.0) {
    if (std::abs(val) < thr) val = 0.0;
    return val;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "admittance_control");
    ros::NodeHandle nh("~");

    double delta_t = 0.01;
    AdmittanceControl ctrl(nh, delta_t);

    Eigen::Vector3d F_ext = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_world_sp = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_world_sp = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_world_sp = Eigen::Vector3d::Zero();
    double yaw_sp;

    bool on_off = false;
    ros::Subscriber traj_start_trigger_pub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/traj_start_trigger",
        100,
        [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            if (msg->header.frame_id == "start") {
                ctrl.initReference();
                p_world_sp = point3MsgToEigen(msg->pose.position);
                a_world_sp = v_world_sp = Eigen::Vector3d::Zero();
                on_off = true;
            } else {
                p_world_sp = point3MsgToEigen(msg->pose.position);
                a_world_sp = v_world_sp = Eigen::Vector3d::Zero();
                on_off = false;
            }
        },
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "odom",
        100,
        [&](const nav_msgs::Odometry::ConstPtr& msg) {
            p_world = point3MsgToEigen(msg->pose.pose.position);
            v_world = vector3MsgToEigen(msg->twist.twist.linear);
        },
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    ros::Subscriber ewe_sub = nh.subscribe<geometry_msgs::WrenchStamped>(
        "ewe",
        100,
        [&](const geometry_msgs::WrenchStamped::ConstPtr& msg) {
            F_ext = vector3MsgToEigen(msg->wrench.force);
            F_ext[0] = remove_noise(F_ext[0], 1.0);
            F_ext[1] = remove_noise(F_ext[1], 1.0);
            F_ext[2] = remove_noise(F_ext[2], 1.0);
            return;
        },
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 100);
    quadrotor_msgs::PositionCommand cmd;

    ros::Rate r(int(1.0 / delta_t));
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();

        if (on_off == false) continue;

        ctrl.Control(F_ext, p_world, v_world, &p_world_sp, &v_world_sp, &a_world_sp, &yaw_sp);
        // a_world_sp -= F_ext;

        cmd.header.frame_id = "world";
        cmd.header.seq++;
        cmd.header.stamp = ros::Time::now();

        cmd.position = eigenToPoint3Msg(p_world_sp);
        cmd.velocity = eigenToVector3Msg(v_world_sp);
        cmd.acceleration = eigenToVector3Msg(a_world_sp);
        cmd.jerk = eigenToVector3Msg(Eigen::Vector3d::Zero());
        cmd.yaw = 0;
        cmd.yaw_dot = 0;
        cmd.yaw = uav_utils::normalize_angle(cmd.yaw);

        cmd_pub.publish(cmd);
    }

    return 0;
}
