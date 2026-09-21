#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <uav_utils/utils.h>

class AdmittanceControl {
   private:
    Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d M_inv = Eigen::Matrix3d::Identity();

   private:
    bool init_rf;
    Eigen::Vector3d p_world_rf;
    Eigen::Vector3d v_world_rf;
    Eigen::Vector3d a_world_rf;

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
        M_inv = M.inverse();

        initReference();
        setDeltaT(dt);
    }

    ~AdmittanceControl() {
        //
    }

    void initReference() {
        init_rf = true;
        return;
    }

    void setDeltaT(double dt) {
        delta_t = dt;
        return;
    }

    void Control(const Eigen::Vector3d F_ext, const Eigen::Vector3d& p_world, const Eigen::Vector3d& v_world, Eigen::Vector3d* const p_world_sp_ptr, Eigen::Vector3d* const v_world_sp_ptr, Eigen::Vector3d* const a_world_sp_ptr, double* const yaw_sp_ptr) {
        Eigen::Vector3d& p_world_sp = (*p_world_sp_ptr);
        Eigen::Vector3d& v_world_sp = (*v_world_sp_ptr);
        Eigen::Vector3d& a_world_sp = (*a_world_sp_ptr);
        // double& yaw_sp = (*yaw_sp_ptr);

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
        // yaw_sp = 0.0;

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

Eigen::Quaterniond quaternionMsgToEigen(const geometry_msgs::Quaternion& msg) { return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z); }

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

bool collision_trigger(Eigen::Vector3d F_ext, double t) {
    static bool collsion = false;

    if (F_ext.norm() > 4.0 && t > 2.0) collsion = true;

    return collsion;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "collision_recovery");
    ros::NodeHandle nh("~");

    double delta_t = 0.01;
    AdmittanceControl ctrl(nh, delta_t);

    Eigen::Vector3d linear_motion_speed = Eigen::Vector3d::Zero();
    ctrl.getParam(nh, "LinearMotionSpeed/Vx", linear_motion_speed.x());
    ctrl.getParam(nh, "LinearMotionSpeed/Vy", linear_motion_speed.y());
    ctrl.getParam(nh, "LinearMotionSpeed/Vz", linear_motion_speed.z());

    double dt_fix_min = 0.2;
    double dt_stop_min = 0.2;
    double F_collision_min = 2.0;
    double fix_vel_decay = 0.95;
    double stop_vel_max = 0.2;

    ctrl.getParam(nh, "Threshold/dt_fix_min", dt_fix_min);
    ctrl.getParam(nh, "Threshold/dt_stop_min", dt_stop_min);
    ctrl.getParam(nh, "Threshold/F_collision_min", F_collision_min);
    ctrl.getParam(nh, "Threshold/fix_vel_decay", fix_vel_decay);
    ctrl.getParam(nh, "Threshold/stop_vel_max", stop_vel_max);

    Eigen::Vector3d F_ext = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_world_sp = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_world_sp = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_world_sp = Eigen::Vector3d::Zero();
    double yaw_sp;

    bool first_collison = false;

    bool on_off = false;
    geometry_msgs::Pose pose;
    ros::Time t0 = ros::Time::now();
    ros::Subscriber traj_start_trigger_pub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/traj_start_trigger",
        100,
        [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            pose = msg->pose;
            p_world_sp = point3MsgToEigen(msg->pose.position);
            a_world_sp = v_world_sp = Eigen::Vector3d::Zero();
            auto q_sp = quaternionMsgToEigen(msg->pose.orientation);
            yaw_sp = uav_utils::get_yaw_from_quaternion(q_sp);

            t0 = ros::Time::now();
            if (msg->header.frame_id == "start") {
                ctrl.initReference();
                on_off = true;
                first_collison = true;
            } else
                on_off = false;
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

    ros::Publisher cwe_pub = nh.advertise<geometry_msgs::WrenchStamped>("/tx_uav/collsion_recovery/cwe", 100);

    ros::Subscriber ewe_sub = nh.subscribe<geometry_msgs::WrenchStamped>(
        "ewe",
        100,
        [&](const geometry_msgs::WrenchStamped::ConstPtr& msg) {
            F_ext = vector3MsgToEigen(msg->wrench.force);
            // F_ext[0] = remove_noise(F_ext[0], 1.0);
            // F_ext[1] = remove_noise(F_ext[1], 1.0);
            // F_ext[2] = remove_noise(F_ext[2], 1.0);
            auto F_fix = F_ext.dot(linear_motion_speed.normalized()) * linear_motion_speed;
            F_ext = 0.8 * F_fix + 0.2 * F_ext;
            collision_trigger(F_ext, (ros::Time::now() - t0).toSec());

            geometry_msgs::WrenchStamped msg_pub = *msg;
            msg_pub.wrench.force = eigenToVector3Msg(F_ext);
            cwe_pub.publish(msg_pub);
            return;
        },
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 100);
    quadrotor_msgs::PositionCommand cmd;

    ros::Publisher collision_pub = nh.advertise<std_msgs::Bool>("/tx_uav/collsion_recovery/collision", 100);
    ros::Publisher fix_pub = nh.advertise<std_msgs::Bool>("/tx_uav/collsion_recovery/fix_pose", 100);
    ros::Publisher stop_pub = nh.advertise<std_msgs::Bool>("/tx_uav/collsion_recovery/stop", 100);

    bool collision = false;
    ros::Time t_collision = t0;

    Eigen::Vector3d position_fix = Eigen::Vector3d::Zero();
    bool fix_pose = false;
    ros::Time t_fix_pose = t0;

    bool stop = false;
    // ros::Time t_stop = t0;

    ros::Rate r(int(1.0 / delta_t));
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();

        if (on_off == false) continue;

        if (stop) {
            a_world_sp = Eigen::Vector3d::Zero();
            v_world_sp = Eigen::Vector3d::Zero();
            p_world_sp = position_fix;
        } else if (fix_pose) {
            a_world_sp = Eigen::Vector3d::Zero();
            // v_world_sp = v_world_sp * 0.95 - linear_motion_speed;
            // linear_motion_speed *= 0.9;
            v_world_sp = v_world_sp * fix_vel_decay;
            p_world_sp = position_fix + v_world_sp * delta_t;

            if ((ros::Time::now() - t_fix_pose).toSec() >= dt_stop_min && v_world_sp.norm() <= stop_vel_max) {
                stop = true;
                a_world_sp = Eigen::Vector3d::Zero();
                v_world_sp = Eigen::Vector3d::Zero();
                position_fix = p_world;
                p_world_sp = position_fix;
            }
        } else {
            collision = collision_trigger(F_ext, (ros::Time::now() - t0).toSec());

            if (collision == true) {
                if (first_collison) {
                    first_collison = false;
                    ctrl.initReference();
                    t_collision = ros::Time::now();
                }

                ctrl.Control(F_ext, p_world, v_world, &p_world_sp, &v_world_sp, &a_world_sp, &yaw_sp);

                if ((ros::Time::now() - t_collision).toSec() >= dt_fix_min && F_ext.norm() <= F_collision_min) {
                    fix_pose = true;
                    position_fix = p_world;
                    p_world_sp = position_fix;
                    t_fix_pose = ros::Time::now();
                }
            } else {
                a_world_sp = Eigen::Vector3d::Zero();
                v_world_sp = linear_motion_speed;
                double dt = (ros::Time::now() - t0).toSec();
                p_world_sp = point3MsgToEigen(pose.position) + v_world_sp * dt;
                // yaw_sp = 0.0;
            }
        }

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

        // For debug
        {
            std_msgs::Bool bool_msg;

            bool_msg.data = collision;
            collision_pub.publish(bool_msg);

            bool_msg.data = fix_pose;
            fix_pub.publish(bool_msg);

            bool_msg.data = stop;
            stop_pub.publish(bool_msg);
        }
    }

    return 0;
}
