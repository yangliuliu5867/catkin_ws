#ifndef TRAJ_GEN_IN_CORRIDOR_HPP
#define TRAJ_GEN_IN_CORRIDOR_HPP

#include "traj_gen_in_corridor/config.hpp"
#include "traj_gen_in_corridor/visualizer.hpp"
#include "traj_gen_in_corridor/tictoc.hpp"
#include "gcopter/solver/geoutils.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/collision_gcopter.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <chrono>
#include <random>
#include <sstream>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <limits>
#include <atomic>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "quadrotor_msgs/CollisionTrajectory.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "quadrotor_msgs/Corridor.h"
#include "quadrotor_msgs/CorridorList.h"
#include "quadrotor_msgs/OptCostDebug.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/TrajServerDebug.h"
#include "quadrotor_msgs/TrajectoryPlan.h"

// Utility: resolve relative data_ CSV paths to workspace-root/data_/filename
static inline std::string ensureTrailingSlashLocalTG(std::string path)
{
    if (path.empty())
        return path;
    if (path.back() != '/')
        path.push_back('/');
    return path;
}

static inline std::string resolveDataCsvPathTrajGen(const std::string &filename, const std::string &data_dir = "data_/")
{
    std::string dir = data_dir.empty() ? std::string("data_/") : data_dir;
    if (!dir.empty() && dir.front() == '/') {
        dir = ensureTrailingSlashLocalTG(dir);
        return dir + filename;
    }
    std::string pkg = ros::package::getPath("traj_gen_in_corridor");
    const std::string needle = "/src/";
    std::string root = pkg;
    auto pos = pkg.rfind(needle);
    if (pos != std::string::npos)
        root = pkg.substr(0, pos);
    if (!root.empty() && root.back() != '/')
        root.push_back('/');
    dir = ensureTrailingSlashLocalTG(dir);
    return root + dir + filename;
}

class GlobalPlanner
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber targetSub;
    ros::Subscriber odomSub;
    ros::Subscriber corridorSub;
    ros::Subscriber intentionSub;
    ros::Subscriber keyPosArraySub;
    ros::Subscriber collision_trajectory_sub_;  // 新增：完整碰撞轨迹订阅器
    // 预测状态订阅器（来自控制端）
    ros::Subscriber pred_pose_sub_;
    ros::Subscriber pred_vel_sub_;
    ros::Subscriber pred_acc_sub_;
    ros::Publisher mapPub, optDebugPub;
    ros::Publisher trajPub, visTrajPub;
    ros::Publisher collisionTrajPub;  // 碰撞轨迹（含碰撞事件）发布给控制端

    std::atomic<bool> corridorInitialized{false};
    std::atomic<bool> odomInitialized{false};
    std::atomic<bool> targetInitialized{false};
    // priority targets: key_pos (from TrajectoryPlan) preferred over /goal
    std::atomic<bool> has_keypos_{false};
    std::atomic<bool> has_goal_{false};
    ros::Time last_keypos_time_;
    ros::Time last_goal_time_;
    Eigen::Vector3d keypos_target_;
    Eigen::Vector3d goal_target_;
    std::atomic<bool> sample_times_available{false};
    std::atomic<bool> use_collision_optimization{false}; // Flag to control which optimizer to use
    std::atomic<bool> has_collision_events_{false};
    std::atomic<bool> trajectory_optimized_{false};      // Flag to track if trajectory has been optimized
    std::atomic<bool> waiting_for_collision_data_{false}; // Flag to indicate waiting for collision data
    Eigen::Vector3d initialPos;
    Eigen::Vector3d finalPos;
    // 预测起始状态（来自控制端预测轨迹），用于保证重规划轨迹在PVA上连续
    geometry_msgs::PoseStamped latest_pred_pose_;
    geometry_msgs::Vector3Stamped latest_pred_vel_;
    geometry_msgs::Vector3Stamped latest_pred_acc_;
    bool pred_initialized_ = false;
    std::mutex pred_mutex_;
    Visualizer visualizer;
    gcopter::GCOPTER gcopter;                           // For collision-free trajectories
    collision_gcopter::COLLISION_GCOPTER collision_gcopter_; // For collision trajectories
    quadrotor_msgs::PolynomialTrajectory trajMsg;

    std::vector<Eigen::Matrix<double, 6, -1>> corridor;
    std::vector<Eigen::Matrix<double, 6, -1>> corridor_inflated; // used for optimization (inflated)
    Eigen::VectorXd sample_times;

    // Latest received pre-collision (TrajectoryPlan) waypoints + segment times (front-end initial guess)
    std::atomic<bool> has_latest_pre_waypoints_{false};
    std::vector<Eigen::Vector3d> latest_pre_waypoints_;
    std::atomic<bool> has_latest_pre_segment_times_{false};
    std::vector<double> latest_pre_segment_times_;

    // Latest received collision trajectory payload (post-collision 5 points from front-end)
    std::atomic<bool> has_latest_collision_post_points_{false};
    std::vector<Eigen::Vector3d> latest_collision_post_points_;
    // Latest received per-segment times for post-collision samples
    std::atomic<bool> has_latest_collision_post_segment_times_{false};
    std::vector<double> latest_collision_post_segment_times_;

    // yaw角目标存储
    std::vector<double> segment_yaw_targets;  // 每个段的目标yaw角序列

    // 碰撞事件存储和处理
    std::vector<collision_gcopter::CollisionEvent> collision_events_;
    // 保存上次可行性检查中观测到的最大速度/加速度，避免重复采样
    double last_v_obs_ = 0.0;
    double last_a_obs_ = 0.0;
    // 保存上次可行性检查时的走廊 RMS 违例（米）
    double last_rms_violation_ = 0.0;

    static inline void inflateCorridorPolyHInPlace(
        std::vector<Eigen::Matrix<double, 6, -1>> &corrs,
        const double inflate_m)
    {
        if (inflate_m <= 0.0)
        {
            return;
        }

        for (auto &poly : corrs)
        {
            for (int i = 0; i < poly.cols(); ++i)
            {
                const Eigen::Vector3d n = poly.col(i).head<3>();
                const double nrm = n.norm();
                if (!std::isfinite(nrm) || nrm < 1e-9)
                {
                    continue;
                }
                Eigen::Vector3d p0 = poly.col(i).tail<3>();
                p0 += (n / nrm) * inflate_m;
                poly.col(i).tail<3>() = p0;
            }
        }
    }


    static inline bool isPointInCorridorPolyH(
        const Eigen::Matrix<double, 6, -1> &corridor_poly,
        const Eigen::Vector3d &pt,
        double eps = 1e-6)
    {
        for (int i = 0; i < corridor_poly.cols(); ++i)
        {
            const Eigen::Vector3d nor = corridor_poly.col(i).head<3>();
            const Eigen::Vector3d p0 = corridor_poly.col(i).tail<3>();
            const double dot = nor.dot(p0 - pt);
            if (dot < -eps)
            {
                return false;
            }
        }
        return true;
    }

    inline std::vector<Eigen::Matrix<double, 6, -1>> selectCorridorsCoveringPoints(
        const std::vector<Eigen::Vector3d> &points,
        const std::vector<Eigen::Matrix<double, 6, -1>> &full_corridor)
    {
        if (full_corridor.empty() || points.empty())
        {
            return full_corridor;
        }

        const int num_corridors = static_cast<int>(full_corridor.size());
        int required_start = num_corridors - 1;
        int required_end = 0;
        bool found_any = false;

        for (const auto &pt : points)
        {
            int idx_found = -1;
            // search from the end for robustness (matches prior heuristic)
            for (int i = num_corridors - 1; i >= 0; --i)
            {
                if (isPointInCorridorPolyH(full_corridor[i], pt))
                {
                    idx_found = i;
                    break;
                }
            }
            if (idx_found >= 0)
            {
                found_any = true;
                required_start = std::min(required_start, idx_found);
                required_end = std::max(required_end, idx_found);
            }
        }

        if (!found_any)
        {
            return full_corridor;
        }
        if (required_end < required_start)
        {
            required_end = required_start;
        }

        std::vector<Eigen::Matrix<double, 6, -1>> selected;
        selected.reserve(required_end - required_start + 1);
        for (int i = required_start; i <= required_end; ++i)
        {
            selected.push_back(full_corridor[i]);
        }
        return selected;
    }


public:
    GlobalPlanner(Config &conf, ros::NodeHandle &nh_)
        : config(conf), nh(nh_), visualizer(config, nh)
    {
        trajPub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>(config.trajTopic, 1);
        visTrajPub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>(config.visTrajTopic, 1);
        // 向控制端发布包含碰撞点位置、碰撞前/后速度和时间信息的轨迹
        // 使用 latched publisher，确保后加入的控制端也能立刻收到最近一次碰撞事件
        collisionTrajPub = nh.advertise<quadrotor_msgs::CollisionTrajectory>("collision_trajectory", 1, true);
        optDebugPub = nh.advertise<quadrotor_msgs::OptCostDebug>("optCostChange", 1);
        odomSub = nh.subscribe(config.odomTopic, 1,
                               &GlobalPlanner::odomCallBack, this,
                               ros::TransportHints().tcpNoDelay());
        targetSub = nh.subscribe(config.targetTopic, 1,
                                 &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
        corridorSub = nh.subscribe("corridor_list", 1,
                                   &GlobalPlanner::corridorCallback, this,
                                 ros::TransportHints().tcpNoDelay());
        intentionSub = nh.subscribe("/MyPointSeq", 1,
                                    &GlobalPlanner::IntentionCallback, this,
                                    ros::TransportHints().tcpNoDelay());
        keyPosArraySub = nh.subscribe("key_pos", 1,
                                     &GlobalPlanner::keyPosArrayCallback, this,
                                     ros::TransportHints().tcpNoDelay());
        collision_trajectory_sub_ = nh.subscribe("complete_collision_trajectory", 1,
                                               &GlobalPlanner::collisionTrajectoryCallback, this,
                                               ros::TransportHints().tcpNoDelay());
        // 订阅控制端发布的预测状态，用于作为新轨迹的起始边界条件
        pred_pose_sub_ = nh.subscribe("trajectory_predicted_pose", 1,
                         &GlobalPlanner::predPoseCallback, this,
                         ros::TransportHints().tcpNoDelay());
        pred_vel_sub_ = nh.subscribe("trajectory_predicted_velocity", 1,
                        &GlobalPlanner::predVelCallback, this,
                        ros::TransportHints().tcpNoDelay());
        pred_acc_sub_ = nh.subscribe("trajectory_predicted_acceleration", 1,
                        &GlobalPlanner::predAccCallback, this,
                        ros::TransportHints().tcpNoDelay());
        
        // 初始化默认yaw角目标（示例）
        initializeDefaultYawTargets();
    }

    inline void IntentionCallback(const quadrotor_msgs::TrajectoryPlan::ConstPtr &msg)
    {
        if (!odomInitialized.load())
        {
            ROS_WARN("No Odom!");
            return;
        }

        if (msg->waypoints.empty())
        {
            ROS_WARN("TrajectoryPlan has no waypoints");
            return;
        }

        const auto &goal_wp = msg->waypoints.back();
        finalPos(0) = goal_wp.x;
        finalPos(1) = goal_wp.y;
        finalPos(2) = std::max(goal_wp.z, 0.5);
        // mark that we have a goal from Intention (/MyPointSeq)
        goal_target_ = Eigen::Vector3d(finalPos(0), finalPos(1), finalPos(2));
        has_goal_ = true;
        last_goal_time_ = ros::Time::now();
        targetInitialized.store(true);
    }

    inline void keyPosArrayCallback(const quadrotor_msgs::TrajectoryPlan::ConstPtr &msg)
    {
        if (msg->waypoints.size() < 2) {
            ROS_WARN("TrajectoryPlan has insufficient waypoints!");
            return;
        }

        // Cache raw pre waypoints from front-end as initial guess for collision optimization
        latest_pre_waypoints_.clear();
        latest_pre_waypoints_.reserve(msg->waypoints.size());
        for (const auto &wp : msg->waypoints)
        {
            latest_pre_waypoints_.emplace_back(wp.x, wp.y, wp.z);
        }
        has_latest_pre_waypoints_.store(latest_pre_waypoints_.size() >= 2);

        // Only accept discrete waypoints + per-segment durations from the front-end.
        if (msg->representation != quadrotor_msgs::TrajectoryPlan::REP_WAYPOINTS) {
            ROS_ERROR_THROTTLE(2.0,
                               "key_pos: REP_BSPLINE is no longer supported in traj_gen_in_corridor. "
                               "Please send REP_WAYPOINTS (discrete waypoints + segment_times, size=waypoints-1)."
            );
            sample_times_available.store(false);
            return;
        }

        if (msg->trajectory_mode == quadrotor_msgs::TrajectoryPlan::MODE_COLLISION) {
            use_collision_optimization.store(true);
            // Mark that we are expecting a CollisionTrajectory from the front-end
            waiting_for_collision_data_.store(true);
            trajectory_optimized_.store(false);
            ROS_INFO("Using collision trajectory optimization (collision_gcopter)");
        } else {
            use_collision_optimization.store(false);
            // Clear any leftover collision state when switching to collision-free mode
            has_collision_events_.store(false);
            collision_events_.clear();
            has_latest_collision_post_points_.store(false);
            latest_collision_post_points_.clear();
            waiting_for_collision_data_.store(false);
            trajectory_optimized_.store(false);
            ROS_INFO("Using collision-free trajectory optimization (gcopter)");
        }

        const int expected_segments = static_cast<int>(msg->waypoints.size()) - 1;
        const int num_segments = static_cast<int>(msg->segment_times.size());
        if (num_segments != expected_segments) {
            ROS_ERROR("key_pos: segment_times size (%d) must equal waypoints-1 (%d) for REP_WAYPOINTS.",
                      num_segments, expected_segments);
            sample_times_available.store(false);
            has_latest_pre_segment_times_.store(false);
            latest_pre_segment_times_.clear();
            return;
        }

        sample_times.resize(expected_segments);
        latest_pre_segment_times_.clear();
        latest_pre_segment_times_.reserve(expected_segments);
        for (int i = 0; i < expected_segments; ++i) {
            const double ti = std::max(1e-6, std::abs(msg->segment_times[i]));
            sample_times(i) = ti;
            latest_pre_segment_times_.push_back(ti);
        }

        has_latest_pre_segment_times_.store(!latest_pre_segment_times_.empty());

        sample_times_available.store(expected_segments > 0);

        // Only set finalPos from the received TrajectoryPlan when we are NOT
        // in collision mode waiting for a CollisionTrajectory. If we are
        // waiting for CollisionTrajectory, that message will provide the
        // authoritative post-collision endpoint.
        if (!(use_collision_optimization.load() && waiting_for_collision_data_.load())) {
            const auto &last_wp = msg->waypoints.back();
            finalPos(0) = last_wp.x;
            finalPos(1) = last_wp.y;
            finalPos(2) = std::max(last_wp.z, 0.5);
            // Record as key_pos target (higher priority)
            keypos_target_ = Eigen::Vector3d(finalPos(0), finalPos(1), finalPos(2));
            has_keypos_.store(true);
        } else {
            ROS_INFO("keyPosArrayCallback: collision mode active and waiting for CollisionTrajectory; deferring finalPos update");
        }
        // Prefer the sent header timestamp when available so we can anchor incoming
        // CollisionTrajectory messages to this key_pos update.
        if (!msg->header.stamp.isZero()) {
            last_keypos_time_ = msg->header.stamp;
        } else {
            last_keypos_time_ = ros::Time::now();
        }
        targetInitialized.store(true);
        ROS_INFO("Target initialized from TrajectoryPlan (key_pos): (%.3f, %.3f, %.3f)", 
             finalPos(0), finalPos(1), finalPos(2));

        // Build combined segment times: pre (from msg->segment_times) + post (from latest_collision_post_segment_times_)
        std::vector<double> combined_times;
        combined_times.reserve(sample_times.size() + latest_collision_post_segment_times_.size());
        for (int i = 0; i < sample_times.size(); ++i) combined_times.push_back(sample_times(i));
        // If post-segment-times are available, append them (these correspond to post-collision samples).
        if (has_latest_collision_post_segment_times_.load()) {
            for (double t : latest_collision_post_segment_times_) combined_times.push_back(t);
        }

        // Format for logging
        std::stringstream ss;
        ss.setf(std::ios::fixed); ss<<std::setprecision(4);
        for (size_t i = 0; i < combined_times.size(); ++i) {
            if (i) ss << ", ";
            ss << combined_times[i];
        }
        // If we are in collision optimization mode and still waiting for the
        // CollisionTrajectory (post) from front-end, defer printing the
        // combined timing here so that the log includes post-segment times.
        if (!(use_collision_optimization.load() && waiting_for_collision_data_.load()) || has_latest_collision_post_segment_times_.load()) {
            ROS_INFO("Received Sample timing (combined): %zu segments, times: [%s]", combined_times.size(), ss.str().c_str());
        } else {
            ROS_INFO("keyPosArrayCallback: collision mode active and waiting for CollisionTrajectory; deferring sample timing log until CollisionTrajectory arrives");
        }

        if (corridorInitialized.load() && !use_collision_optimization.load()) {
            ROS_INFO("keyPosArrayCallback: corridor ready, applying temporary corridor filter and triggering traj_generator() (key_pos)");

            // Compute filtered subset covering start->end
            Eigen::Vector3d start_pt = initialPos;
            Eigen::Vector3d end_pt;
            end_pt(0) = finalPos(0);
            end_pt(1) = finalPos(1);
            end_pt(2) = finalPos(2);

            std::vector<Eigen::Matrix<double, 6, -1>> filtered = selectCorridorsCoveringPoints(latest_pre_waypoints_, corridor);

            // Limit number of corridors to keep for optimizer (conservative default)
            const size_t kMaxKeep = 10;
            if (filtered.size() > kMaxKeep) filtered.resize(kMaxKeep);

            // Keep uninflated corridors for visualization, create inflated copy for optimization
            corridor = filtered;
            corridor_inflated = filtered;
            inflateCorridorPolyHInPlace(corridor_inflated, 0.2);

            traj_generator();
        }
    }

    inline void corridorCallback(const quadrotor_msgs::CorridorList::ConstPtr &msg)
    {
        corridor.clear();
        int totalNum = msg->corridor_cnt;
        for (int i = 0; i < totalNum; i++)
        {
            int size = msg->corridor_list[i].size;
            Eigen::Matrix<double, 6, -1> hP(6, size);
            if (msg->corridor_type == quadrotor_msgs::CorridorList::CORRIDOR_TYPE_H)
            {
                for (int j = 0; j < size; j++)
                {
                    hP.col(j)[0] = msg->corridor_list[i].nom_vec_list[j].x;
                    hP.col(j)[1] = msg->corridor_list[i].nom_vec_list[j].y;
                    hP.col(j)[2] = msg->corridor_list[i].nom_vec_list[j].z;

                    hP.col(j)[3] = msg->corridor_list[i].point_list[j].x;
                    hP.col(j)[4] = msg->corridor_list[i].point_list[j].y;
                    hP.col(j)[5] = msg->corridor_list[i].point_list[j].z;
                }
            }
            else
            {
                ROS_ERROR("Not ready for V reprensetion \n");
            }
            corridor.push_back(hP);
        }
        // Keep uninflated corridors for visualization, create inflated copy for optimization
        corridor_inflated = corridor;
        inflateCorridorPolyHInPlace(corridor_inflated, 0.2);
        corridorInitialized.store(true);

        // 不自动触发优化：只设置状态，等待显式请求（例如前端发布 key_pos 或用户触发）
        // 保留碰撞轨迹时等待碰撞事件的逻辑
        if (use_collision_optimization.load()) {
            waiting_for_collision_data_.store(true);
            ROS_INFO("Corridor ready for collision trajectory. Waiting for collision events data...");
        } else {
            // 不自动触发 collision-free 优化，记录状态供外部触发使用
            ROS_INFO("Corridor ready for collision-free trajectory. Auto-trigger disabled; waiting for explicit key_pos / user request.");
        }
    }

    inline void initializeCorridor(std::string path, int id)
    {
        std::ifstream fin(path);
        int totalNum = 0;
        fin >> totalNum;

        corridor.clear();
        for (int i = 0; i < totalNum; i++)
        {
            Eigen::Matrix<double, 6, -1> hP;
            int H_num = 0;
            fin >> H_num;
            hP.resize(6, H_num);
            for (int j = 0; j < H_num; j++)
            {
                fin >> hP(0, j) >> hP(1, j) >> hP(2, j) >> hP(3, j) >> hP(4, j) >> hP(5, j);
            }
                corridor.push_back(hP);
            }
            fin.close();
            // Keep uninflated corridors for visualization, create inflated copy for optimization
            corridor_inflated = corridor;
            inflateCorridorPolyHInPlace(corridor_inflated, 0.2);
            corridorInitialized.store(true);
        ros::Duration(2.0).sleep();
        traj_generator();
    }

    inline void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg)
    {
        initialPos(0) = msg->pose.pose.position.x;
        initialPos(1) = msg->pose.pose.position.y;
        initialPos(2) = msg->pose.pose.position.z;
        odomInitialized.store(true);
    }

    // === 预测状态回调：由控制端提供的当前预测起点PVA，用于轨迹起始边界 ===
    inline void predPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(pred_mutex_);
        latest_pred_pose_ = *msg;
        pred_initialized_ = true;
    }

    inline void predVelCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(pred_mutex_);
        latest_pred_vel_ = *msg;
        pred_initialized_ = true;
    }

    inline void predAccCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(pred_mutex_);
        latest_pred_acc_ = *msg;
        pred_initialized_ = true;
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        finalPos(0) = msg->pose.position.x;
        finalPos(1) = msg->pose.position.y;
        finalPos(2) = std::max(msg->pose.position.z, 0.5);
        // record as /goal (lower priority than key_pos)
        goal_target_ = Eigen::Vector3d(finalPos(0), finalPos(1), finalPos(2));
        has_goal_.store(true);
        last_goal_time_ = ros::Time::now();
        targetInitialized.store(true);
    }

    inline void genPolyTrajMsg(const Trajectory<TRAJ_ORDER> &traj,
                               const Eigen::Isometry3d &tfR2L,
                               const ros::Time &iniStamp,
                               quadrotor_msgs::PolynomialTrajectory &trajMsg)
    {
        // 自动根据轨迹段数设置yaw角目标（如果尚未设置）
        int num_segments = traj.getPieceNum();
        bool collision_yaw_mode = use_collision_optimization.load() && has_collision_events_.load() && !collision_events_.empty();
        double collision_time = collision_yaw_mode ? collision_events_.front().collision_time : -1.0;
        const double total_duration = traj.getTotalDuration();
        // 碰撞时间无效则降级为普通模式
        if (!collision_yaw_mode || collision_time <= 0.0 || collision_time >= total_duration) {
            collision_yaw_mode = false;
        }

        if (!collision_yaw_mode) {
            if (segment_yaw_targets.empty()) {
                ROS_INFO("No yaw targets provided for %d segments; yaw coefficients will be omitted (controller falls back to velocity-based yaw)", num_segments);
            } else if ((int)segment_yaw_targets.size() < num_segments + 1) {
                ROS_WARN("Insufficient yaw targets (%d) for %d segments. Using default yaw = 0.0", 
                         (int)segment_yaw_targets.size(), num_segments);
            }
        } else {
            ROS_INFO("Collision trajectory: yaw will keep front-end target before collision, then slowly follow velocity after t=%.3f", collision_time);
        }
        
        trajMsg.header.stamp = iniStamp;
        static uint32_t traj_id = 0;
        traj_id++;
        trajMsg.trajectory_id = traj_id;
        trajMsg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
        trajMsg.num_order = traj[0].getDegree();
        trajMsg.num_segment = traj.getPieceNum();
        Eigen::Vector3d initialVel, finalVel;
        initialVel = tfR2L * traj.getVel(0.0);
        finalVel = tfR2L * traj.getVel(traj.getTotalDuration());
        trajMsg.start_yaw = 0.0;
        trajMsg.final_yaw = 0.0;

        // Ensure yaw coefficient vector is empty by default; will be populated based on mode.
        trajMsg.coef_yaw.clear();

        // yaw 跟随参数：碰撞后跟随速度方向，刻意设置较慢的跟随速率
        const double vel_yaw_thresh = 0.12;   // m/s，低于该值保持前一朝向
        const double max_yaw_follow_rate = 0.20; // rad/s，慢速跟随

        auto unwrap = [](double ang) {
            while (ang > M_PI) ang -= 2.0 * M_PI;
            while (ang < -M_PI) ang += 2.0 * M_PI;
            return ang;
        };

        double prev_yaw = 0.0;
        if (!segment_yaw_targets.empty()) {
            prev_yaw = getSegmentStartYaw(0);
        } else {
            Eigen::Vector3d v0 = tfR2L * traj.getVel(0.0);
            if (v0.head<2>().norm() > vel_yaw_thresh) prev_yaw = atan2(v0.y(), v0.x());
        }

        for (size_t p = 0; p < (size_t)traj.getPieceNum(); p++)
        {
            const double seg_duration = traj[p].getDuration();
            trajMsg.time.push_back(seg_duration);
            trajMsg.order.push_back(traj[p].getCoeffMat().cols() - 1);

            Eigen::VectorXd linearTr(2);
            linearTr << 0.0, trajMsg.time[p];
            std::vector<Eigen::VectorXd> linearTrCoeffs;
            linearTrCoeffs.emplace_back(1);
            linearTrCoeffs[0] << 1;
            for (size_t k = 0; k < trajMsg.order[p]; k++)
            {
                linearTrCoeffs.push_back(RootFinder::polyConv(linearTrCoeffs[k], linearTr));
            }

            Eigen::MatrixXd coefMat(3, traj[p].getCoeffMat().cols());
            for (int i = 0; i < coefMat.cols(); i++)
            {
                coefMat.col(i) = tfR2L.rotation() * traj[p].getCoeffMat().col(coefMat.cols() - i - 1).head<3>();
            }
            coefMat.col(0) = (coefMat.col(0) + tfR2L.translation()).eval();

            for (int i = 0; i < coefMat.cols(); i++)
            {
                double coefx(0.0), coefy(0.0), coefz(0.0);
                for (int j = i; j < coefMat.cols(); j++)
                {
                    coefx += coefMat(0, j) * linearTrCoeffs[j](i);
                    coefy += coefMat(1, j) * linearTrCoeffs[j](i);
                    coefz += coefMat(2, j) * linearTrCoeffs[j](i);
                }
                trajMsg.coef_x.push_back(coefx);
                trajMsg.coef_y.push_back(coefy);
                trajMsg.coef_z.push_back(coefz);
            }
            
            // 生成yaw角多项式系数
            int coeff_cols = traj[p].getCoeffMat().cols();
            int poly_order = std::max(0, coeff_cols - 1);  // 确保至少为0
            
            if (coeff_cols <= 0) {
                ROS_ERROR("Invalid coefficient matrix size for segment %d: cols=%d, using default", (int)p, coeff_cols);
                poly_order = 0;  // 使用最简单的常数多项式
            }
            
            std::vector<double> yaw_coeffs(poly_order + 1, 0.0);

            // ==== yaw 生成策略 ====
            if (!collision_yaw_mode) {
                // 避碰/无碰撞：保持原有策略（仅当前端提供 yaw 目标时填充）
                if (!segment_yaw_targets.empty()) {
                    double segment_start_yaw = getSegmentStartYaw(p);
                    double segment_end_yaw = getSegmentEndYaw(p);

                    yaw_coeffs[0] = segment_start_yaw;
                    if (poly_order >= 1 && seg_duration > 1e-6) {
                        yaw_coeffs[1] = (segment_end_yaw - segment_start_yaw) / seg_duration;
                    }
                    for (size_t i = 0; i < yaw_coeffs.size(); i++) {
                        trajMsg.coef_yaw.push_back(yaw_coeffs[i]);
                    }
                    prev_yaw = segment_end_yaw;
                }
                // segment_yaw_targets 为空则不填充 yaw，控制端按速度方向
            } else {
                // 碰撞轨迹：碰撞前用前端yaw，碰撞后缓慢跟随速度方向
                const double seg_start_time = std::accumulate(trajMsg.time.begin(), trajMsg.time.end(), 0.0) - seg_duration;
                const double seg_end_time = seg_start_time + seg_duration;

                double y0 = prev_yaw;
                double y1 = prev_yaw;

                if (seg_end_time <= collision_time) {
                    // 碰撞前：保持/插值前端给定 yaw（若有）；否则保持当前 yaw 不变
                    if (!segment_yaw_targets.empty()) {
                        y0 = getSegmentStartYaw(p);
                        y1 = getSegmentEndYaw(p);
                    }
                } else if (seg_start_time >= collision_time) {
                    // 碰撞后：根据速度方向缓慢跟随
                    Eigen::Vector3d vel_end = tfR2L * traj.getVel(seg_end_time);
                    double heading = (vel_end.head<2>().norm() > vel_yaw_thresh) ? atan2(vel_end.y(), vel_end.x()) : prev_yaw;
                    double delta = unwrap(heading - y0);
                    double max_delta = max_yaw_follow_rate * seg_duration;
                    double clamped = std::max(-max_delta, std::min(max_delta, delta));
                    y1 = y0 + clamped;
                } else {
                    // 跨越碰撞点的段：前半段保持前端yaw，后半段缓慢跟随速度
                    double before_dur = collision_time - seg_start_time;
                    double after_dur = seg_duration - before_dur;

                    double y_collision = y0;
                    if (!segment_yaw_targets.empty() && seg_duration > 1e-6) {
                        double y_end_target = getSegmentEndYaw(p);
                        y_collision = y0 + (y_end_target - y0) * (before_dur / seg_duration);
                    }

                    Eigen::Vector3d vel_end = tfR2L * traj.getVel(seg_end_time);
                    double heading = (vel_end.head<2>().norm() > vel_yaw_thresh) ? atan2(vel_end.y(), vel_end.x()) : y_collision;
                    double delta = unwrap(heading - y_collision);
                    double max_delta = max_yaw_follow_rate * after_dur;
                    double clamped = std::max(-max_delta, std::min(max_delta, delta));
                    y1 = y_collision + clamped;
                }

                yaw_coeffs[0] = y0;
                if (poly_order >= 1 && seg_duration > 1e-6) {
                    yaw_coeffs[1] = (y1 - y0) / seg_duration;
                }

                // 碰撞轨迹：始终填充 yaw 系数，保证控制端按我们规划执行
                for (size_t i = 0; i < yaw_coeffs.size(); i++) {
                    trajMsg.coef_yaw.push_back(yaw_coeffs[i]);
                }
                prev_yaw = y1;
            }
        }

        trajMsg.mag_coeff = 1.0;
        trajMsg.debug_info = "";
    }    

    // 快速可行性检查：稀疏采样轨迹，检查动力学和走廊约束（宽松阈值用于实时）
    inline bool isTrajectoryFeasible(const Trajectory<TRAJ_ORDER> &traj,
                                     double constraint_tol = 5e-3,
                                     double vel_tol = 0.05,
                                     double acc_tol = 0.5)
    {
        // 若无轨迹或为空，视为不可行
        if (traj.getPieceNum() <= 0) return false;

        double T = traj.getTotalDuration();
        if (T <= 0.0) return false;

        // 采样数：至少50点，按50Hz采样
        int samples = std::max(50, (int)std::ceil(T * 50.0));
        double v_obs = 0.0, a_obs = 0.0;
        bool dynamics_failed = false;

        for (int i = 0; i <= samples; ++i) {
            double t = T * (double(i) / double(samples));
            Eigen::Vector3d pos = traj.getPos(t);
            Eigen::Vector3d vel = traj.getVel(t);
            Eigen::Vector3d acc = traj.getAcc(t);

            // record observed maxima during this single sampling pass
            v_obs = std::max(v_obs, vel.norm());
            a_obs = std::max(a_obs, acc.norm());

            // 动力学约束检查（仅记录失败，不提前返回，以确保收集完整的观测极值）
            if (vel.norm() > config.maxVelRate + vel_tol) {
                dynamics_failed = true;
            }
            if (acc.norm() > config.maxAccRate + acc_tol) {
                dynamics_failed = true;
            }

            // 走廊约束检查（软约束实现）：
            // 计算每个采样点相对于走廊的违反量（若点位于任一多面体内，viol=0），
            // 累积 RMS 违例量并与阈值比较；同时保留硬阈值拒绝过大违例。
            if (!corridor.empty()) {
                double point_min_violation = std::numeric_limits<double>::infinity();
                for (const auto &poly : corridor) {
                    // 对当前多面体，计算该点相对于所有半空间的最大“越界量”
                    double poly_violation = 0.0; // 0 表示在多面体内
                    for (int j = 0; j < poly.cols(); ++j) {
                        Eigen::Vector3d nor = poly.col(j).head<3>();
                        Eigen::Vector3d pt = poly.col(j).tail<3>();
                        double d = nor.dot(pt - pos); // >=0 表示满足该半空间
                        double viol = 0.0;
                        if (d < 0.0) viol = -d; // 负值表示越界，viol 是正的越界深度
                        if (viol > poly_violation) poly_violation = viol;
                        // small early exit: if poly_violation already exceeds some large hard limit, break
                        if (poly_violation > 1.0) break;
                    }
                    if (poly_violation < point_min_violation) point_min_violation = poly_violation;
                    if (point_min_violation <= 0.0) break; // 已经在某个多面体内，完全满足
                }

                // 记录违例量（点到最近多面体的越界深度，单位为米）
                // 累积用于总体 RMS 判断；将在后续单独累积计算 RMS
            }
        }
        // 记录本次可行性检查时观测到的最大速度/加速度，供后续发布路径复用，避免重复采样
        last_v_obs_ = v_obs;
        last_a_obs_ = a_obs;
        // 默认重置 RMS 记录（后续第二遍会更新为真实值）
        last_rms_violation_ = 0.0;

        // 第二遍：重新计算违例 RMS（单独累积以便判定）
        double violation_accum_sq = 0.0;
        int violation_count = 0;

        for (int i = 0; i <= samples; ++i) {
            double t = T * (double(i) / double(samples));
            Eigen::Vector3d pos = traj.getPos(t);

            if (corridor.empty()) continue;

            double point_min_violation = std::numeric_limits<double>::infinity();
            for (const auto &poly : corridor) {
                double poly_violation = 0.0;
                for (int j = 0; j < poly.cols(); ++j) {
                    Eigen::Vector3d nor = poly.col(j).head<3>();
                    Eigen::Vector3d pt = poly.col(j).tail<3>();
                    double d = nor.dot(pt - pos);
                    double viol = (d < 0.0) ? -d : 0.0;
                    if (viol > poly_violation) poly_violation = viol;
                    if (poly_violation > 1.0) break;
                }
                if (poly_violation < point_min_violation) point_min_violation = poly_violation;
                if (point_min_violation <= 0.0) break;
            }

            if (point_min_violation == std::numeric_limits<double>::infinity()) point_min_violation = 0.0;

            violation_accum_sq += point_min_violation * point_min_violation;
            ++violation_count;

            // 硬阈值：如果某一点严重越界（例如 > 0.2m），记录并直接判不可行
            const double hard_violation_thresh = 0.20; // meters
            if (point_min_violation > hard_violation_thresh) {
                // 记录严重违例量，供发布逻辑区分 RMS 超限与动力学超限
                last_rms_violation_ = point_min_violation;
                ROS_WARN_THROTTLE(2.0, "Trajectory infeasible: severe corridor violation %.3fm at t=%.3f", point_min_violation, t);
                return false;
            }
        }

        if (violation_count > 0) {
            double rms = std::sqrt(violation_accum_sq / (double)violation_count);
            // 记录本次可行性检查的 RMS 违例量（米）供后续决策使用
            last_rms_violation_ = rms;
            if (rms > constraint_tol) {
                ROS_WARN_THROTTLE(2.0, "Trajectory infeasible: corridor RMS violation %.6fm > tol %.6fm", rms, constraint_tol);
                return false;
            } else {
                ROS_DEBUG_THROTTLE(2.0, "Trajectory corridor RMS violation OK: %.6fm <= tol %.6fm", rms, constraint_tol);
            }
        }

        if (dynamics_failed) {
            ROS_WARN_THROTTLE(2.0, "Trajectory infeasible: dynamics exceeded limit (max_v=%.3f, max_a=%.3f)", last_v_obs_, last_a_obs_);
            return false;
        }

        return true;
    }

    // 设置yaw角目标序列
    inline void setYawTargets(const std::vector<double>& yaw_targets)
    {
        segment_yaw_targets = yaw_targets;
        ROS_INFO("Set %d yaw angle targets for trajectory segments", (int)yaw_targets.size());
    }
    
    // 获取指定段的起始yaw角
    inline double getSegmentStartYaw(int segment_index) const
    {
        if (segment_yaw_targets.empty())
        {
            // 如果没有设置yaw角目标，使用全局start_yaw
            return 0.0;  // 默认值
        }
        
        if (segment_index < 0 || segment_index >= (int)segment_yaw_targets.size())
        {
            ROS_WARN("Invalid segment index %d for yaw targets (size: %d)", 
                     segment_index, (int)segment_yaw_targets.size());
            return 0.0;
        }
        
        return segment_yaw_targets[segment_index];
    }
    
        // 获取指定段的结束yaw角
    inline double getSegmentEndYaw(int segment_index) const
    {
        if (segment_yaw_targets.empty())
        {
            // 如果没有设置yaw角目标，使用默认值
            return 0.0;  // 默认值
        }
        
        if (segment_index < 0)
        {
            ROS_WARN("Invalid segment index %d (negative)", segment_index);
            return 0.0;
        }
        
        // 对于N个段，我们需要N+1个yaw角值：[yaw0, yaw1, ..., yawN]
        // 段i的结束yaw角 = segment_yaw_targets[i+1]
        int end_yaw_index = segment_index + 1;
        
        if (end_yaw_index < (int)segment_yaw_targets.size())
        {
            return segment_yaw_targets[end_yaw_index];
        }
        else
        {
            // 如果没有足够的yaw目标值，使用起始yaw角（保持不变）
            if (segment_index < (int)segment_yaw_targets.size())
            {
                return segment_yaw_targets[segment_index];
            }
            else
            {
                ROS_WARN("Segment index %d out of range for yaw targets (size: %d)", 
                         segment_index, (int)segment_yaw_targets.size());
                return 0.0;
            }
        }
    }
     
     // 初始化默认yaw角目标（动态适应轨迹段数）
     inline void initializeDefaultYawTargets()
     {
         // 不设置固定的yaw目标，保持为空
         // 这样所有段的yaw角都会默认为0
         segment_yaw_targets.clear();
         ROS_INFO("Initialized default yaw targets: all segments yaw = 0.0");
     }
     
     // 根据轨迹段数动态设置yaw角目标
     inline void setYawTargetsForSegments(int num_segments)
     {
         if (num_segments <= 0) {
             ROS_WARN("Invalid number of segments: %d", num_segments);
             return;
         }
         
         // 生成均匀分布的yaw角目标（从0到90度）
         std::vector<double> yaw_targets;
         for (int i = 0; i <= num_segments; i++) {
             double yaw_deg = (double)i / num_segments * 90.0;  // 0到90度均匀分布
             yaw_targets.push_back(yaw_deg * M_PI / 180.0);  // 转换为弧度
         }
         
         setYawTargets(yaw_targets);
         ROS_INFO("Set %d yaw targets for %d segments", (int)yaw_targets.size(), num_segments);
     }
     
     // 便捷函数：按度数设置yaw角目标
     inline void setYawTargetsDegrees(const std::vector<double>& yaw_degrees)
     {
         std::vector<double> yaw_radians;
         for (double deg : yaw_degrees)
         {
             yaw_radians.push_back(deg * M_PI / 180.0);
         }
         setYawTargets(yaw_radians);
    }

    // Load trajectory from file
    inline bool load_trajectory(const std::string& filename, Trajectory<TRAJ_ORDER>& traj)
    {
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open trajectory file: %s", filename.c_str());
            return false;
        }

        int num_segments;
        file >> num_segments;
        
        if (num_segments <= 0) {
            ROS_ERROR("Invalid number of segments: %d", num_segments);
            return false;
        }

        std::vector<double> durations(num_segments);
        std::vector<typename Piece<TRAJ_ORDER>::CoefficientMat> coeffMats(num_segments);

        for (int i = 0; i < num_segments; i++) {
            // Read duration
            file >> durations[i];
            
            // Read coefficient matrix (3 x (TRAJ_ORDER + 1))
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col <= TRAJ_ORDER; col++) {
                    file >> coeffMats[i](row, col);
                }
            }
        }

        file.close();

        // Create trajectory
        traj = Trajectory<TRAJ_ORDER>(durations, coeffMats);
        
        ROS_INFO("Successfully loaded trajectory with %d segments from %s", num_segments, filename.c_str());
        return true;
    }

    // 导出轨迹为 CSV（按等步长 dt 采样，默认 0.01s）
    inline bool exportTrajectoryCSV(const std::string& filename, const Trajectory<TRAJ_ORDER>& traj, double dt = 0.01)
    {
        if (traj.getPieceNum() <= 0) {
            ROS_ERROR("exportTrajectoryCSV: empty trajectory");
            return false;
        }

        double T = traj.getTotalDuration();
        if (T <= 0.0) {
            ROS_ERROR("exportTrajectoryCSV: non-positive total duration");
            return false;
        }

        std::ofstream ofs(filename);
        if (!ofs.is_open()) {
            ROS_ERROR("exportTrajectoryCSV: failed to open file %s", filename.c_str());
            return false;
        }

        ofs << "time,px,py,pz,vx,vy,vz,ax,ay,az\n";
        ofs << std::fixed << std::setprecision(6);

        int samples = std::max(1, (int)std::ceil(T / dt));
        for (int i = 0; i <= samples; ++i) {
            double t = std::min(T, dt * (double)i);
            Eigen::Vector3d p = traj.getPos(t);
            Eigen::Vector3d v = traj.getVel(t);
            Eigen::Vector3d a = traj.getAcc(t);
            ofs << t << ","
                << p.x() << "," << p.y() << "," << p.z() << ","
                << v.x() << "," << v.y() << "," << v.z() << ","
                << a.x() << "," << a.y() << "," << a.z() << "\n";
        }

        ofs.close();
        ROS_INFO("exportTrajectoryCSV: wrote %d samples to %s", samples + 1, filename.c_str());
        return true;
    }

    inline void traj_generator()
    {
        if (corridorInitialized.load() && targetInitialized.load())
        {
            // Check sample timing information availability
            if (sample_times_available.load() && sample_times.size() > 0) {
                ROS_INFO("Using sample  timing information for MINCO optimization");
                ROS_INFO("Total sample segments: %d, Total duration: %.3f seconds", 
                         (int)sample_times.size(), sample_times.sum());
            } else {
                ROS_WARN("sample timing information not available, using geometric distance-based timing");
            }
            
            visualizer.visualizePolytope(corridor, 0);

            Eigen::Matrix<double, 3, 4> iniState;
            Eigen::Matrix<double, 3, 4> finState;

            iniState.setZero();
            finState.setZero();

            // 终点：按原逻辑选择 key_pos / goal / finalPos，速度加速度为0
            if (odomInitialized)
            {
                const double KEYPOS_VALID_SECS = 1.0; // seconds
                Eigen::Vector3d chosenFinal = finalPos;
                ros::Time now = ros::Time::now();

                bool keypos_recent = false;
                if (has_keypos_.load()) {
                    double dt = (now - last_keypos_time_).toSec();
                    if (dt >= 0.0 && dt <= KEYPOS_VALID_SECS) keypos_recent = true;
                }

                if (keypos_recent) {
                    chosenFinal = keypos_target_;
                    ROS_INFO("traj_generator: using key_pos as final target (recent %.3fs): (%.3f, %.3f, %.3f)",
                             (now - last_keypos_time_).toSec(), chosenFinal.x(), chosenFinal.y(), chosenFinal.z());
                } else if (has_goal_.load()) {
                    chosenFinal = goal_target_;
                    ROS_INFO("traj_generator: using goal as final target: (%.3f, %.3f, %.3f)", chosenFinal.x(), chosenFinal.y(), chosenFinal.z());
                } else {
                    ROS_WARN_THROTTLE(5.0, "traj_generator: no key_pos or goal recent; using fallback finalPos (%.3f, %.3f, %.3f)", finalPos(0), finalPos(1), finalPos(2));
                }

                finState.col(0) = chosenFinal;
            }

            // 起点：优先使用控制端预测的 PVA，保证轨迹在动力学上的连续
            {
                Eigen::Vector3d pos = Eigen::Vector3d::Zero();
                Eigen::Vector3d vel = Eigen::Vector3d::Zero();
                Eigen::Vector3d acc = Eigen::Vector3d::Zero();

                bool use_pred = false;
                if (pred_initialized_)
                {
                    std::lock_guard<std::mutex> lock(pred_mutex_);
                    pos << latest_pred_pose_.pose.position.x,
                           latest_pred_pose_.pose.position.y,
                           latest_pred_pose_.pose.position.z;
                    vel << latest_pred_vel_.vector.x,
                           latest_pred_vel_.vector.y,
                           latest_pred_vel_.vector.z;
                    acc << latest_pred_acc_.vector.x,
                           latest_pred_acc_.vector.y,
                           latest_pred_acc_.vector.z;

                    use_pred = true;
                }

                if (!use_pred)
                {
                    // 若预测状态不可用，则退回到当前 odom 位置，速度/加速度为0
                    if (odomInitialized.load())
                    {
                        pos = initialPos;
                    }
                    ROS_WARN_THROTTLE(2.0, "Predicted PVA not available, using odom position with zero vel/acc as start state");
                }

                iniState.col(0) = pos;
                iniState.col(1) = vel;
                iniState.col(2) = acc;
                iniState.col(3).setZero(); // jerk 未提供，置0
            }

            double res = INFINITY;
            int itg = config.quadratureResolution;
            int smp = 10;

            // Initialize trajectory planner parameters
            Eigen::VectorXd magnitudeBounds(4);
            Eigen::VectorXd penaltyWeights(8);
            
            magnitudeBounds(0) = config.maxVelRate;
            magnitudeBounds(1) = config.maxAccRate;
            magnitudeBounds(2) = config.maxTau;
            magnitudeBounds(3) = config.minTau;
            
            penaltyWeights(0) = config.chiVec[0];
            penaltyWeights(1) = config.chiVec[1];
            penaltyWeights(2) = config.chiVec[2];
            penaltyWeights(3) = config.chiVec[3];
            penaltyWeights(4) = 0.0;
            penaltyWeights(5) = 0.0;
            penaltyWeights(6) = config.chiVec[6];
            penaltyWeights(7) = config.chiVec[4];

            // Simple placeholder matrices for compatibility
            Eigen::VectorXd dummyUseKeyPos = Eigen::VectorXd::Zero(1);
            Eigen::Matrix3Xd dummyKeyAtt = Eigen::Matrix3Xd::Zero(3, 1);
            Eigen::Matrix3Xd dummyKeyPos = Eigen::Matrix3Xd::Zero(3, 1);
            Eigen::VectorXd dummyKeyTime = Eigen::VectorXd::Zero(1);

            // Sanitize sample timing to avoid feeding non-positive/invalid piece times into optimizer.
            Eigen::VectorXd setup_sample_times;
            if (sample_times_available.load() && sample_times.size() > 0)
            {
                bool finite = sample_times.allFinite();
                bool positive = true;
                for (int i = 0; i < sample_times.size(); ++i)
                {
                    if (sample_times(i) <= 0.0)
                    {
                        positive = false;
                        break;
                    }
                }

                if (finite && positive)
                {
                    setup_sample_times = sample_times;
                    const double kMinPieceTime = 0.05;
                    for (int i = 0; i < setup_sample_times.size(); ++i)
                    {
                        if (setup_sample_times(i) < kMinPieceTime)
                        {
                            setup_sample_times(i) = kMinPieceTime;
                        }
                    }
                }
                else
                {
                    ROS_WARN("sample_times contains non-finite/non-positive values; ignore provided timing and use geometric initialization");
                }
            }

            TicToc timer;
            Trajectory<TRAJ_ORDER> traj;

            // Select optimizer based on trajectory type
            bool optimization_success = false;
            if (use_collision_optimization.load()) {
                ROS_INFO("Using collision trajectory optimizer (collision_gcopter) - delegating to multi-segment generator");
                // Delegate to the multi-segment generator which contains the
                // joint optimization path (pre+post) and detailed collision handling.
                traj_generator_multi_segment();
                return;
            } else {
                ROS_INFO("Using collision-free trajectory optimizer (gcopter)");
                
                // Setup gcopter
            if (!gcopter.setup(config.weightT, iniState, finState,
                               corridor, res, config.smoothingEps, itg, smp,
                               magnitudeBounds, penaltyWeights, dummyUseKeyPos,
                               dummyKeyAtt, dummyKeyPos, dummyKeyTime, config.isDebug,
                               false, visualizer, optDebugPub, setup_sample_times))
            {
                    ROS_ERROR("Collision-free trajectory optimizer setup failed!");
                return;
            }

            // Optimize with gcopter
            int optFailedCount = 0;
            printf("\nOptimizing trajectory, please wait ...\n");
            auto gcopter_failed = [&traj](double cost) {
                return (!std::isfinite(cost)) || (cost < 0.0) || (traj.getPieceNum() <= 0);
            };

            double opt_cost = gcopter.optimize(traj, config.relCostTol);
            while (gcopter_failed(opt_cost) && ros::ok() && optFailedCount < 10)
            {
                optFailedCount += 1;
                printf("Planning attempts is %d.\n\n", optFailedCount);
                printf("Attempting to replan, please be patient ...\n");
                gcopter.setup(config.weightT, iniState, finState,
                               corridor, res, config.smoothingEps, itg, smp,
                               magnitudeBounds, penaltyWeights, dummyUseKeyPos,
                               dummyKeyAtt, dummyKeyPos, dummyKeyTime, config.isDebug,
                               false, visualizer, optDebugPub, setup_sample_times);
                opt_cost = gcopter.optimize(traj, config.relCostTol);
            }
            optimization_success = !gcopter_failed(opt_cost);
            }

            if (!optimization_success){
                printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
                printf("          Unable to find a reasonable trajectory!\n");
                printf("  Please change intentions or constraints then try again!\n");
                printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
                return;
            }

            double dtmsec = timer.toc();

            // Compute the trajectory length (reduced sampling for real-time)
            int segnum = std::min(10000, std::max(100, int(traj.getTotalDuration() * 100.0)));
            double dt = traj.getTotalDuration() / segnum;
            Eigen::Vector3d xk, xk1 = traj.getPos(0.0);
            double length = 0.0;
            for (int i = 0; i < segnum; i++)
            {
                xk = xk1;
                xk1 = traj.getPos(dt * (i + 1.0));
                length += (xk1 - xk).norm();
            }

            // Display trajectory profile
            printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
            printf("Spatial-temporal trajectory optimization completed.\nTrajectory profile:\n");
            printf("       Total   computation  time: %7.2lf msecs.\n", dtmsec);
            printf("       Total   flight   duration: %7.2lf secs.\n", traj.getTotalDuration());
            printf("       Total  trajectory  length: %7.2lf m.\n", length);
            printf("       Average          velocity: %7.2lf m/s.\n", length / traj.getTotalDuration());
            printf("       Max  velocity   magnitude: %7.2lf m/s.\n", traj.getMaxVelRate());
            printf("       Max accleration magnitude: %7.2lf m/s^2.\n", traj.getMaxAccRate());
            printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

            // 如果存在碰撞事件，打印优化后的碰撞前/碰撞后速度信息，并准备发送给控制端
            std::vector<quadrotor_msgs::CollisionEvent> optimized_collision_events;
            if (use_collision_optimization.load() && has_collision_events_.load() && !collision_events_.empty()) {
                size_t collision_count = collision_events_.size();
                double totalT = traj.getTotalDuration();
                printf("---- Collision velocities (after optimization) ----\n");
                for (size_t ci = 0; ci < collision_count && ci < (size_t)traj.getPieceNum(); ++ci) {
                    // collision at end of segment ci -> time = sum durations of segments 0..ci
                    double tcol = 0.0;
                    for (int s = 0; s <= (int)ci; ++s) tcol += traj[s].getDuration();
                    if (tcol > totalT) tcol = totalT;
                    double eps = 1e-6;
                    double tpre = std::max(0.0, tcol - eps);
                    double tpost = std::min(totalT, tcol + eps);

                    Eigen::Vector3d pre_vel = traj.getVel(tpre);
                    Eigen::Vector3d post_vel = collision_gcopter_.getDynamicPostCollisionVelocity(ci);
                    // 如果 collision_gcopter 没有提供缓存的后碰撞速度，尝试从轨迹取后一点速度
                    if (!post_vel.allFinite() || post_vel.norm() < 1e-9) {
                        if (tpost <= totalT) post_vel = traj.getVel(tpost);
                    }

                    printf("Collision %zu at t=%.3f s: pre_vel=(%.3f, %.3f, %.3f), post_vel=(%.3f, %.3f, %.3f)\n",
                           ci, tcol,
                           pre_vel.x(), pre_vel.y(), pre_vel.z(),
                           post_vel.x(), post_vel.y(), post_vel.z());

                    // 构造发送给控制端的碰撞事件（时间基于优化后轨迹）
                    quadrotor_msgs::CollisionEvent ev_msg;
                    const auto &src_ev = collision_events_[ci];
                    // Use optimized (relaxed) collision point sampled from the optimized trajectory
                    // at the computed collision time tcol, instead of the original sample collision point.
                    Eigen::Vector3d optimized_col_pt = traj.getPos(tcol);
                    ev_msg.collision_point.x = optimized_col_pt.x();
                    ev_msg.collision_point.y = optimized_col_pt.y();
                    ev_msg.collision_point.z = optimized_col_pt.z();

                    ev_msg.pre_collision_velocity.x = pre_vel.x();
                    ev_msg.pre_collision_velocity.y = pre_vel.y();
                    ev_msg.pre_collision_velocity.z = pre_vel.z();

                    ev_msg.post_collision_velocity.x = post_vel.x();
                    ev_msg.post_collision_velocity.y = post_vel.y();
                    ev_msg.post_collision_velocity.z = post_vel.z();

                    ev_msg.plane_normal.x = src_ev.plane_normal.x();
                    ev_msg.plane_normal.y = src_ev.plane_normal.y();
                    ev_msg.plane_normal.z = src_ev.plane_normal.z();
                    ev_msg.plane_d = src_ev.plane_d;

                    // transmit backend's given_yaw (if present in internal event)
                    ev_msg.given_yaw = src_ev.given_yaw;

                    // 使用优化后轨迹上的碰撞时间（相对于轨迹起点）
                    ev_msg.collision_time = tcol;

                    optimized_collision_events.push_back(ev_msg);
                }
                printf("-----------------------------------------------\n\n");
            }

            if (traj.getPieceNum() > 0)
            {
                visualizer.visualize(traj, 0, true);
                int samples = int(length / 0.1);
                visualizer.visualizeEllipsoid(traj, samples);

                // 导出优化后单段轨迹为 CSV，便于离线分析（0.01s 采样）
                try {
                    std::string csv_path = resolveDataCsvPathTrajGen(std::string("optimized_traj.csv"));
                    exportTrajectoryCSV(csv_path, traj, 0.01);
                    ROS_INFO("Exported optimized_traj to %s", csv_path.c_str());
                } catch (...) {
                    ROS_WARN("Failed to export optimized_traj.csv (exception)");
                }
                
                // 在发布前进行快速可行性检查（宽松阈值）
                double constraint_tol = 5e-2; // 5 cm
                double vel_tol = 0.05;        // 5 cm/s
                double acc_tol = 0.5;         // 0.5 m/s^2

                bool feasible = isTrajectoryFeasible(traj, constraint_tol, vel_tol, acc_tol);

                const double v_lim = config.maxVelRate;
                const double a_lim = config.maxAccRate;
                const double k_max = 1.2;
                const double safety = 1.0;

                bool allow_time_scaled_publish = false;
                if (feasible) {
                    allow_time_scaled_publish = true;
                } else if (last_rms_violation_ <= constraint_tol && (last_v_obs_ > v_lim || last_a_obs_ > a_lim)) {
                    // 仅在走廊 RMS 在容忍范围内但存在动力学超限时，允许一次基于观测值的时间缩放并发布。
                    allow_time_scaled_publish = true;
                    ROS_WARN("Trajectory dynamics overshoot detected but RMS within tol (%.6fm); will apply single-pass time-scaling", last_rms_violation_);
                } else {
                    ROS_WARN("Trajectory failed feasibility check: RMS violation %.6fm, v_obs=%.3f (lim %.3f), a_obs=%.3f (lim %.3f)",
                             last_rms_violation_, last_v_obs_, v_lim, last_a_obs_, a_lim);
                }

                if (!allow_time_scaled_publish) {
                    ROS_WARN("Optimized trajectory failed feasibility check - skipping publish");
                } else {
                    // Reuse maxima observed during feasibility check to avoid a second sampling pass
                    double v_obs = last_v_obs_;
                    double a_obs = last_a_obs_;
                    double k = 1.0;
                    if (v_obs > v_lim) {
                        k = (v_obs / v_lim) * safety;
                    } else if (a_obs > a_lim) {
                        k = std::sqrt(a_obs / a_lim) * safety;
                    }
                    if (k < 1.0) k = 1.0;
                    if (k > k_max) k = k_max;

                    if (k > 1.0001) {
                        ROS_WARN("Applying time-scaling k=%.3f (v_obs=%.3f a_obs=%.3f) to avoid small overshoot", k, v_obs, a_obs);
                    }

                    ros::Time unified_start_time = ros::Time::now() + ros::Duration(config.traj_start_delay);
                    quadrotor_msgs::PolynomialTrajectory vistrajMsg;
                    genPolyTrajMsg(traj, Eigen::Isometry3d::Identity(), unified_start_time, vistrajMsg);

                    // Apply time scaling only to per-segment times
                    if (k > 1.0001) {
                        for (size_t ti = 0; ti < vistrajMsg.time.size(); ++ti) {
                            vistrajMsg.time[ti] = vistrajMsg.time[ti] * k;
                        }
                    }

                    trajMsg = vistrajMsg;
                    trajPub.publish(trajMsg);
                    visTrajPub.publish(trajMsg);

                    // 如果存在优化后的碰撞事件，将其作为 CollisionTrajectory 发布给控制端
                    if (use_collision_optimization.load() && !optimized_collision_events.empty() && collisionTrajPub) {
                        quadrotor_msgs::CollisionTrajectory col_traj_msg;
                        col_traj_msg.header.stamp = unified_start_time;
                        col_traj_msg.header.frame_id = "world";

                        col_traj_msg.collision_events = optimized_collision_events;

                        collisionTrajPub.publish(col_traj_msg);
                        ROS_INFO("Published %zu optimized collision events to controller", optimized_collision_events.size());
                        trajectory_optimized_.store(true);
                    } else if (!use_collision_optimization.load() && collisionTrajPub) {
                        quadrotor_msgs::CollisionTrajectory col_traj_msg;
                        col_traj_msg.header.stamp = unified_start_time;
                        col_traj_msg.header.frame_id = "world";
                        collisionTrajPub.publish(col_traj_msg);
                    }
                }
            }
        }
        return;
    }

    // Load and execute trajectory from file
    inline bool load_and_execute_trajectory(const std::string& filename)
    {
        Trajectory<TRAJ_ORDER> loaded_traj;
        
        // Load trajectory from file
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open trajectory file: %s", filename.c_str());
            return false;
        }

        int num_segments;
        file >> num_segments;
        
        if (num_segments <= 0) {
            ROS_ERROR("Invalid number of segments: %d", num_segments);
            return false;
        }

        std::vector<double> durations(num_segments);
        std::vector<typename Piece<TRAJ_ORDER>::CoefficientMat> coeffMats(num_segments);

        for (int i = 0; i < num_segments; i++) {
            // Read duration
            file >> durations[i];
            
            // Read coefficient matrix (3 x (TRAJ_ORDER + 1))
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col <= TRAJ_ORDER; col++) {
                    file >> coeffMats[i](row, col);
                }
            }
        }

        file.close();

        // Create trajectory
        loaded_traj = Trajectory<TRAJ_ORDER>(durations, coeffMats);
        
        ROS_INFO("Successfully loaded trajectory with %d segments from %s", num_segments, filename.c_str());

        // Execute the loaded trajectory
        if (loaded_traj.getPieceNum() > 0)
        {
            // Visualize trajectory
            visualizer.visualize(loaded_traj, 0, true);
            int samples = int(loaded_traj.getTotalDuration() * 10); // 10 Hz sampling
            visualizer.visualizeEllipsoid(loaded_traj, samples);
            
            // Generate trajectory message with current time as start
            ros::Time start_time = ros::Time::now() + ros::Duration(config.traj_start_delay); // Small delay for safety
            
            quadrotor_msgs::PolynomialTrajectory loadedTrajMsg;
            genPolyTrajMsg(loaded_traj, Eigen::Isometry3d::Identity(), start_time, loadedTrajMsg);
            
            // Publish trajectory to controller
            trajPub.publish(loadedTrajMsg);
            visTrajPub.publish(loadedTrajMsg);

            if (collisionTrajPub) {
                quadrotor_msgs::CollisionTrajectory col_traj_msg;
                col_traj_msg.header.stamp = start_time;
                col_traj_msg.header.frame_id = "world";
                collisionTrajPub.publish(col_traj_msg);
            }
            
            ROS_INFO("Executing loaded trajectory with %d segments, duration: %.2fs", 
                     loaded_traj.getPieceNum(), loaded_traj.getTotalDuration());
            return true;
        }
        
        ROS_ERROR("Failed to execute empty trajectory");
        return false;
    }

    // === 新增：完整碰撞轨迹回调函数 ===
    inline void collisionTrajectoryCallback(const quadrotor_msgs::CollisionTrajectory::ConstPtr &msg)
    {
        // Compute pre (from latest sample_times) and post (from message) point counts
        size_t post_points = msg->trajectory_points.size();
        int pre_points = 0;
        if (sample_times_available.load()) {
            pre_points = static_cast<int>(sample_times.size()) + 1; // segments -> waypoints
        }
        int merged_points = pre_points + static_cast<int>(post_points);
        // If both exist, assume overlap of the collision point (deduplicate one)
        if (pre_points > 0 && post_points > 0) merged_points -= 1;

        ROS_INFO("Received CollisionTrajectory: post=%zu, pre=%d, merged=%d, collision_events=%zu",
                 post_points, pre_points, merged_points, msg->collision_events.size());

        // Quick gating: only consume CollisionTrajectory when we are explicitly
        // waiting for collision data (set by keyPosArrayCallback). This avoids
        // reacting to stale/out-of-context messages from earlier runs.
        ros::Time msg_time = (!msg->header.stamp.isZero()) ? msg->header.stamp : ros::Time::now();
        if (!waiting_for_collision_data_.load()) {
            ROS_WARN_THROTTLE(2.0, "collisionTrajectoryCallback: not waiting for collision data, ignoring message");
            return;
        }

        double dt = (msg_time - last_keypos_time_).toSec();
        if (dt < -0.1 || dt > 5.0) {
            ROS_WARN("collisionTrajectoryCallback: message timestamp dt=%.3f outside allowed window relative to last_keypos_time_, ignoring", dt);
            return;
        }

        collision_events_.clear();
        has_collision_events_.store(false);

        latest_collision_post_points_.clear();
        latest_collision_post_points_.reserve(msg->trajectory_points.size());
        for (const auto &p : msg->trajectory_points)
        {
            latest_collision_post_points_.emplace_back(p.x, p.y, p.z);
        }
        has_latest_collision_post_points_.store(latest_collision_post_points_.size() >= 2);

        // read segment times if provided
        latest_collision_post_segment_times_.clear();
        latest_collision_post_segment_times_.reserve(msg->segment_times.size());
        for (const auto &t : msg->segment_times) {
            latest_collision_post_segment_times_.push_back(t);
        }
        has_latest_collision_post_segment_times_.store(!latest_collision_post_segment_times_.empty());

        // If we are waiting for collision data in collision mode, log the
        // combined pre+post sample timings now that post segment times arrived.
        if (use_collision_optimization.load() && waiting_for_collision_data_.load()) {
            std::vector<double> combined_times;
            if (sample_times_available.load()) {
                for (int i = 0; i < sample_times.size(); ++i) combined_times.push_back(sample_times(i));
            }
            if (has_latest_collision_post_segment_times_.load()) {
                for (double t : latest_collision_post_segment_times_) combined_times.push_back(t);
            }
            std::stringstream ss2;
            ss2.setf(std::ios::fixed); ss2<<std::setprecision(4);
            for (size_t i = 0; i < combined_times.size(); ++i) {
                if (i) ss2 << ", ";
                ss2 << combined_times[i];
            }
            ROS_INFO("CollisionTrajectory: Combined Sample timing (pre+post): %zu segments, times: [%s]", combined_times.size(), ss2.str().c_str());
        }

        // If this CollisionTrajectory includes post-collision trajectory points,
        // treat its last point as the intended final target for collision mode.
        if (!latest_collision_post_points_.empty()) {
            const Eigen::Vector3d &last_pt = latest_collision_post_points_.back();
            finalPos(0) = last_pt.x();
            finalPos(1) = last_pt.y();
            finalPos(2) = std::max(last_pt.z(), 0.5);
            keypos_target_ = Eigen::Vector3d(finalPos(0), finalPos(1), finalPos(2));
            has_keypos_.store(true);
            targetInitialized.store(true);
            ROS_INFO("collisionTrajectoryCallback: set finalPos from CollisionTrajectory end: (%.3f, %.3f, %.3f)", finalPos(0), finalPos(1), finalPos(2));
        }

        std::vector<collision_gcopter::CollisionEvent> extracted_collision_events;
        extracted_collision_events.reserve(msg->collision_events.size());

        for (const auto &ev_msg : msg->collision_events) {
            collision_gcopter::CollisionEvent ev;
            ev.collision_point = Eigen::Vector3d(ev_msg.collision_point.x,
                                                 ev_msg.collision_point.y,
                                                 ev_msg.collision_point.z);
            ev.collision_time = ev_msg.collision_time;
            ev.pre_collision_velocity = Eigen::Vector3d(ev_msg.pre_collision_velocity.x,
                                                        ev_msg.pre_collision_velocity.y,
                                                        ev_msg.pre_collision_velocity.z);
            ev.post_collision_velocity = Eigen::Vector3d(ev_msg.post_collision_velocity.x,
                                                         ev_msg.post_collision_velocity.y,
                                                         ev_msg.post_collision_velocity.z);
            ev.plane_normal = Eigen::Vector3d(ev_msg.plane_normal.x,
                                              ev_msg.plane_normal.y,
                                              ev_msg.plane_normal.z);
            ev.plane_d = ev_msg.plane_d;

            // 规范化平面法向并修正速度的法向分量符号：
            // - pre_collision_velocity 的法向分量应为负（朝向平面，approach）
            // - post_collision_velocity 的法向分量应为正（远离平面）
            if (ev.plane_normal.norm() > 1e-9) {
                ev.plane_normal.normalize();

                if (ev.pre_collision_velocity.norm() > 1e-9) {
                    double pre_n = ev.pre_collision_velocity.dot(ev.plane_normal);
                    Eigen::Vector3d pre_t = ev.pre_collision_velocity - pre_n * ev.plane_normal;
                    if (pre_n > 0.0) pre_n = -pre_n; // 保证朝向平面为负
                    ev.pre_collision_velocity = pre_t + pre_n * ev.plane_normal;
                }

                if (ev.post_collision_velocity.norm() > 1e-9) {
                    double post_n = ev.post_collision_velocity.dot(ev.plane_normal);
                    Eigen::Vector3d post_t = ev.post_collision_velocity - post_n * ev.plane_normal;
                    if (post_n < 0.0) post_n = -post_n; // 保证远离平面为正
                    ev.post_collision_velocity = post_t + post_n * ev.plane_normal;
                }
            }

            const bool has_pre_velocity = (ev.pre_collision_velocity.norm() > 1e-6);
            const bool has_post_velocity = (ev.post_collision_velocity.norm() > 1e-6);
            const bool has_normal = (ev.plane_normal.norm() > 1e-6);

            if (has_pre_velocity && has_post_velocity && has_normal) {
                extracted_collision_events.push_back(ev);
                ROS_INFO("Collision event: point=(%.3f,%.3f,%.3f), time=%.3f",
                         ev.collision_point.x(), ev.collision_point.y(), ev.collision_point.z(), ev.collision_time);
                ROS_INFO("  Pre-collision velocity: (%.3f,%.3f,%.3f)",
                         ev.pre_collision_velocity.x(), ev.pre_collision_velocity.y(), ev.pre_collision_velocity.z());
                ROS_INFO("  Post-collision velocity: (%.3f,%.3f,%.3f)",
                         ev.post_collision_velocity.x(), ev.post_collision_velocity.y(), ev.post_collision_velocity.z());
                ROS_INFO("  Plane normal: (%.3f,%.3f,%.3f), d=%.3f",
                         ev.plane_normal.x(), ev.plane_normal.y(), ev.plane_normal.z(), ev.plane_d);
            } else {
                ROS_WARN("Incomplete collision event at time %.3f: pre_vel=%s, post_vel=%s, normal=%s",
                         ev.collision_time,
                         has_pre_velocity ? "YES" : "NO",
                         has_post_velocity ? "YES" : "NO",
                         has_normal ? "YES" : "NO");
            }
        }

        if (!extracted_collision_events.empty()) {
            if (extracted_collision_events.size() > 1) {
                ROS_WARN("Received %lu collision events, single-collision mode keeps only the first one.",
                         extracted_collision_events.size());
            }

            collision_events_.push_back(extracted_collision_events.front());
            has_collision_events_.store(true);
            use_collision_optimization.store(true);

            ROS_INFO("Successfully extracted %lu collision events from CollisionTrajectory", collision_events_.size());
            ROS_INFO("Switched to collision optimization mode");

            // A CollisionTrajectory message indicates a new collision scenario from the front-end.
            // Always reset the optimized flag so we don't accidentally skip due to stale state.
            trajectory_optimized_.store(false);

            // Trigger optimization as soon as corridor + target are ready.
            if (corridorInitialized.load() && targetInitialized.load()) {
                waiting_for_collision_data_.store(false);
                ROS_INFO("All data ready, triggering collision trajectory optimization");
                traj_generator();
            } else {
                waiting_for_collision_data_.store(true);
                ROS_INFO("Waiting for corridor and target data before optimizing collision trajectory");
            }
        } else {
            ROS_INFO("No valid collision events found - using collision-free optimization");
            use_collision_optimization.store(false);
            has_collision_events_.store(false);
        }

        ROS_INFO("CollisionTrajectory processing finished: %lu trajectory points, %lu collision events (kept=%lu)",
             latest_collision_post_points_.size(), extracted_collision_events.size(), collision_events_.size());
    }

    // ===== 碰撞处理相关函数 =====
    
    // 主要接口：从sample规划器获取数据并生成轨迹
    template<typename KinodynamicSampleRosPtr>
    inline void processSampleResult(KinodynamicSampleRosPtr sample_planner)
    {
        if (!sample_planner) {
            ROS_ERROR("Invalid sample planner pointer");
            return;
        }
        
        ROS_INFO("Processing sample planning result...");
        
        // 1. 从sample获取碰撞数据
        setCollisionDataFromsample(sample_planner);
        
        // 2. 自动选择轨迹类型并生成
        if (has_collision_events_.load() && !collision_events_.empty()) {
            use_collision_optimization.store(true);
            ROS_INFO("Collision events detected, using collision trajectory optimization");
        } else {
            use_collision_optimization.store(false);
            ROS_INFO("No collision events, using collision-free trajectory optimization");
        }
        
        // 3. 执行轨迹生成
        traj_generator();
    }
    
    // 接收来自sample的碰撞事件信息
    inline void setCollisionEvents(const std::vector<collision_gcopter::CollisionEvent>& events)
    {
        collision_events_.clear();
        if (!events.empty()) {
            if (events.size() > 1) {
                ROS_WARN("Received %lu collision events, single-collision mode keeps only the first one.", events.size());
            }
            collision_events_.push_back(events.front());
        }
        has_collision_events_.store(!collision_events_.empty());

        if (has_collision_events_.load()) {
            ROS_INFO("Received %lu collision events from sample:", events.size());
            for (size_t i = 0; i < events.size(); ++i) {
                const auto& event = events[i];
                ROS_INFO("  Event %lu: collision at (%.3f, %.3f, %.3f), time: %.3f", 
                         i + 1, event.collision_point.x(), event.collision_point.y(), 
                         event.collision_point.z(), event.collision_time);
            }
        } else {
            ROS_INFO("No collision events received - using collision-free optimization");
        }
    }

    // 从sample碰撞轨迹中提取真实碰撞事件（过滤虚拟碰撞点）
    template<typename CollisionsampleNodePtr>
    inline void extractAndFilterCollisionEvents(const std::vector<CollisionsampleNodePtr>& collision_tree_nodes)
    {
        collision_events_.clear();
        
        if (collision_tree_nodes.empty()) {
            has_collision_events_.store(false);
            ROS_INFO("No collision tree nodes provided");
            return;
        }
        
        ROS_INFO("Processing %lu collision tree nodes to extract real collision events", collision_tree_nodes.size());
        
        // 遍历碰撞树节点，过滤出真实的碰撞事件
        for (const auto& collision_node : collision_tree_nodes) {
            const auto& collision_primitive = collision_node->collision_primitive;
            
            // 过滤虚拟碰撞点：检查碰撞点坐标是否为非零（真实碰撞）
            bool is_real_collision = (collision_primitive.collision_info.collision_point.norm() > 1e-6);
            
            if (is_real_collision) {
                collision_gcopter::CollisionEvent event;
                event.collision_point = collision_primitive.collision_info.collision_point;
                event.plane_normal = collision_primitive.collision_info.plane_normal;
                event.plane_d = collision_primitive.collision_info.plane_d;
                event.collision_time = collision_primitive.collision_info.collision_time;
                
                // 直接使用sample提供的碰撞前后速度（sample已经处理了反弹模型）
                event.pre_collision_velocity = collision_primitive.collision_state.velocity;
                
                // sample计算的反弹后速度（已考虑恢复系数和损失系数）
                if (!collision_primitive.bounce_traj.velocity_profile.empty()) {
                    event.post_collision_velocity = collision_primitive.bounce_traj.start_state.velocity;
                } else {
                    // 备用：使用碰撞状态计算的反弹速度
                    event.post_collision_velocity = collision_primitive.post_collision_traj.start_state.velocity;
                }
                
                collision_events_.push_back(event);
                
                ROS_INFO("Extracted real collision event at (%.3f, %.3f, %.3f)", 
                         event.collision_point.x(), event.collision_point.y(), event.collision_point.z());
                ROS_INFO("  Pre-collision velocity: (%.3f, %.3f, %.3f)", 
                         event.pre_collision_velocity.x(), event.pre_collision_velocity.y(), event.pre_collision_velocity.z());
                ROS_INFO("  Post-collision velocity: (%.3f, %.3f, %.3f)", 
                         event.post_collision_velocity.x(), event.post_collision_velocity.y(), event.post_collision_velocity.z());
            }
        }
        
        has_collision_events_.store(!collision_events_.empty());
        ROS_INFO("Extracted %lu real collision events from %lu tree nodes", 
                 collision_events_.size(), collision_tree_nodes.size());
        
        if (!has_collision_events_.load()) {
            ROS_INFO("No real collision events found - using collision-free optimization");
        }
    }

    // 主要接口：直接从kinodynamic_sample_ros获取碰撞轨迹数据
    template<typename KinodynamicsampleRosPtr>
    inline void setCollisionDataFromsample(KinodynamicsampleRosPtr sample_planner)
    {
        if (!sample_planner) {
            ROS_ERROR("Invalid sample planner pointer");
            has_collision_events_.store(false);
            return;
        }
        
        ROS_INFO("Extracting collision events from sample planner...");
        
        // 检查sample是否有碰撞路径结果
        try {
            auto sample_instance = sample_planner->sample_planner_;
                if (!sample_instance) {
                ROS_WARN("sample planner instance is null");
                has_collision_events_.store(false);
                collision_events_.clear();
                return;
            }
            
            // 获取碰撞树节点并过滤真实碰撞事件
            auto collision_tree_nodes = sample_instance->collision_tree_nodes;
            if (collision_tree_nodes.empty()) {
                ROS_INFO("No collision tree nodes found in sample result");
                has_collision_events_.store(false);
                collision_events_.clear();
                return;
            }
            
            // 提取并过滤碰撞事件
            extractAndFilterCollisionEvents(collision_tree_nodes);

            // 单碰撞模式：仅保留首个事件
            if (collision_events_.size() > 1) {
                ROS_WARN("sample provided %lu collision events, keeping only the first one (single-collision mode).",
                         collision_events_.size());
                collision_events_.erase(collision_events_.begin() + 1, collision_events_.end());
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception while extracting collision data from sample: %s", e.what());
            has_collision_events_.store(false);
            collision_events_.clear();
        }
    }

    // 从碰撞事件创建轨迹段
    inline std::vector<collision_gcopter::TrajectorySegment> createTrajectorySegments()
    {
        std::vector<collision_gcopter::TrajectorySegment> segments;
        
        if (!odomInitialized.load() || !targetInitialized.load()) {
            ROS_ERROR("Cannot create trajectory segments: start or goal not initialized");
            return segments;
        }

        // 构建全局起点和终点状态（起点优先使用控制端预测PVA，保证连续性）
        Eigen::Matrix<double, 3, 4> globalStartState, globalGoalState;
        globalStartState.setZero();
        globalGoalState.setZero();
        {
            Eigen::Vector3d pos = Eigen::Vector3d::Zero();
            Eigen::Vector3d vel = Eigen::Vector3d::Zero();
            Eigen::Vector3d acc = Eigen::Vector3d::Zero();

            bool use_pred = false;
            if (pred_initialized_)
            {
                std::lock_guard<std::mutex> lock(pred_mutex_);
                pos << latest_pred_pose_.pose.position.x,
                    latest_pred_pose_.pose.position.y,
                    latest_pred_pose_.pose.position.z;
                vel << latest_pred_vel_.vector.x,
                    latest_pred_vel_.vector.y,
                    latest_pred_vel_.vector.z;
                acc << latest_pred_acc_.vector.x,
                    latest_pred_acc_.vector.y,
                    latest_pred_acc_.vector.z;
                use_pred = true;
            }

            if (!use_pred)
            {
                if (odomInitialized.load())
                {
                    pos = initialPos;
                }
                ROS_WARN_THROTTLE(2.0, "createTrajectorySegments: predicted PVA not available, using odom position with zero vel/acc");
            }

            globalStartState.col(0) = pos;
            globalStartState.col(1) = vel;
            globalStartState.col(2) = acc;
            globalStartState.col(3).setZero();
        }
        globalGoalState.col(0) = finalPos;
        
        // 如果没有碰撞事件，创建单段轨迹
        if (!has_collision_events_.load() || collision_events_.empty()) {
            collision_gcopter::TrajectorySegment segment;
            segment.start_PVAJ = globalStartState;
            segment.end_PVAJ = globalGoalState;
            segment.corridor = corridor;
            segment.is_collision_segment = false;
            segments.push_back(segment);
            ROS_INFO("No collision events, using single-segment optimization");
            return segments;
        }

        // === 正确的串行优化分段策略：碰撞发生在每段的末端 ===
        size_t num_collision_events = collision_events_.size();
        ROS_INFO("Creating %lu trajectory segments for %lu collision events (collision at segment end)", 
                 num_collision_events + 1, num_collision_events);

        // 使用配置参数初始化碰撞事件的约束参数，并将前端 yaw 传入事件
        for (size_t idx = 0; idx < collision_events_.size(); ++idx) {
            collision_gcopter::CollisionEvent& mutable_event = const_cast<collision_gcopter::CollisionEvent&>(collision_events_[idx]);
            // 配置事件级别的最大位置偏移与前端给定的 yaw（若有）
            mutable_event.max_position_offset = config.max_position_offset;
            // 将前端 yaw 目标（若存在）传入事件
            if (!segment_yaw_targets.empty() && idx < segment_yaw_targets.size()) {
                mutable_event.given_yaw = getSegmentEndYaw(static_cast<int>(idx));
            } else {
                mutable_event.given_yaw = 0.0;
            }
            // 初始化约束参数
            mutable_event.initializeConstraintParameters();
        }

        // === 新的分段策略：每段的碰撞发生在该段末端 ===
        
        // 起点 → 碰撞点（碰撞在该段末端）
        {
            collision_gcopter::TrajectorySegment segment;
            segment.start_PVAJ = globalStartState;
            segment.end_PVAJ.setZero();
            
            // 直接使用碰撞事件信息设置该段的终点状态（碰撞发生在该段末端）
            segment.end_PVAJ.col(0) = collision_events_[0].collision_point;
            segment.end_PVAJ.col(1) = collision_events_[0].pre_collision_velocity;
            segment.end_PVAJ.col(2).setZero();
            segment.end_PVAJ.col(3).setZero();
            
            // 智能选择必要的走廊：使用pre点裁剪走廊（从膨胀副本中选择以用于优化）
            segment.corridor = selectCorridorsCoveringPoints(latest_pre_waypoints_, corridor_inflated);
            
            segment.is_collision_segment = true;  // 碰撞段
            segment.collision_event = collision_events_[0];

            // Initial guess for pre segment from front-end TrajectoryPlan (if available)
            if (has_latest_pre_waypoints_.load() && has_latest_pre_segment_times_.load())
            {
                segment.initial_path_waypoints.resize(3, (int)latest_pre_waypoints_.size());
                for (size_t k = 0; k < latest_pre_waypoints_.size(); ++k)
                {
                    segment.initial_path_waypoints.col((int)k) = latest_pre_waypoints_[k];
                }
                segment.initial_segment_times.resize((int)latest_pre_segment_times_.size());
                for (size_t k = 0; k < latest_pre_segment_times_.size(); ++k)
                {
                    segment.initial_segment_times((int)k) = std::max(1e-6, std::abs(latest_pre_segment_times_[k]));
                }
            }
            
            // === 关键修复：碰撞发生在该段轨迹的末端 ===
            // 需要在优化完成后设置正确的相对时间，这里先设置为占位符
            segment.collision_event.collision_time = -1.0;  // 占位符，表示"段末端"
            
            segments.push_back(segment);
            
            ROS_INFO("Segment 1 (to collision): start=(%.3f,%.3f,%.3f) -> collision=(%.3f,%.3f,%.3f), end_vel=(%.3f,%.3f,%.3f), using %lu corridors",
                     segment.start_PVAJ(0,0), segment.start_PVAJ(1,0), segment.start_PVAJ(2,0),
                     segment.end_PVAJ(0,0), segment.end_PVAJ(1,0), segment.end_PVAJ(2,0),
                     segment.end_PVAJ(0,1), segment.end_PVAJ(1,1), segment.end_PVAJ(2,1),
                     segment.corridor.size());
            ROS_INFO("  Collision occurs at segment END (will be adjusted after optimization)");
        }

        // 后段：碰撞点 → 终点
        {
            collision_gcopter::TrajectorySegment segment;
            segment.start_PVAJ.setZero();
            segment.start_PVAJ.col(0) = collision_events_[num_collision_events - 1].collision_point;
            segment.start_PVAJ.col(1) = collision_events_[num_collision_events - 1].post_collision_velocity;
            
            segment.end_PVAJ = globalGoalState;
            
            // 智能选择必要的走廊：优先用前端提供的 post 段覆盖走廊（使用膨胀副本以用于优化）
            segment.corridor = selectCorridorsCoveringPoints(latest_collision_post_points_, corridor_inflated);
            
            segment.is_collision_segment = false;  // 非碰撞段

            // Initial guess for post segment from front-end CollisionTrajectory (if available)
            if (has_latest_collision_post_points_.load() && has_latest_collision_post_segment_times_.load())
            {
                segment.initial_path_waypoints.resize(3, (int)latest_collision_post_points_.size());
                for (size_t k = 0; k < latest_collision_post_points_.size(); ++k)
                {
                    segment.initial_path_waypoints.col((int)k) = latest_collision_post_points_[k];
                }
                segment.initial_segment_times.resize((int)latest_collision_post_segment_times_.size());
                for (size_t k = 0; k < latest_collision_post_segment_times_.size(); ++k)
                {
                    segment.initial_segment_times((int)k) = std::max(1e-6, std::abs(latest_collision_post_segment_times_[k]));
                }
            }
            
            segments.push_back(segment);
            
            ROS_INFO("Segment %lu (collision to end): start=(%.3f,%.3f,%.3f) -> end=(%.3f,%.3f,%.3f), using %lu corridors",
                     num_collision_events + 1, segment.start_PVAJ(0,0), segment.start_PVAJ(1,0), segment.start_PVAJ(2,0),
                     segment.end_PVAJ(0,0), segment.end_PVAJ(1,0), segment.end_PVAJ(2,0),
                     segment.corridor.size());
        }

        ROS_INFO("Created %lu trajectory segments for collision-aware optimization", segments.size());
        ROS_INFO("Strategy: Each collision occurs at the END of its corresponding segment");
        return segments;
    }

    // 使用多段优化的轨迹生成器
    inline void traj_generator_multi_segment()
    {
        if (!corridorInitialized.load() || !targetInitialized.load()) {
            ROS_WARN("Cannot generate trajectory: corridor or target not initialized");
            return;
        }

        // 创建轨迹段
        auto segments = createTrajectorySegments();
        if (segments.empty()) {
            ROS_ERROR("Failed to create trajectory segments");
            return;
        }

        // 设置优化参数
        Eigen::VectorXd magnitudeBounds(4);
        Eigen::VectorXd penaltyWeights(8);
        
        magnitudeBounds(0) = config.maxVelRate;
        magnitudeBounds(1) = config.maxAccRate;
        magnitudeBounds(2) = config.maxTau;
        magnitudeBounds(3) = config.minTau;
        
        penaltyWeights(0) = config.chiVec[0];
        penaltyWeights(1) = config.chiVec[1];
        penaltyWeights(2) = config.chiVec[2];
        penaltyWeights(3) = config.chiVec[3];
        penaltyWeights(4) = 0.0;
        penaltyWeights(5) = 0.0;
        penaltyWeights(6) = config.chiVec[6];
        penaltyWeights(7) = config.chiVec[4];

        TicToc timer;
        std::vector<Trajectory<TRAJ_ORDER>> segment_trajectories;
        
        // 多段轨迹优化
        bool success = collision_gcopter_.optimizeMultiSegmentTrajectory(
            segments, config.relCostTol, config.weightT, INFINITY, 
            config.smoothingEps, config.quadratureResolution, 10,
            magnitudeBounds, penaltyWeights, config.isDebug,
            visualizer, optDebugPub, segment_trajectories,
            config.position_constraint_weight, config.velocity_constraint_weight,
            config.friction, config.damping_ratio,
            config.max_collision_velocity);

        if (!success) {
            ROS_ERROR("Multi-segment trajectory optimization failed");
            return;
        }

        // 拼接轨迹（pre + post 两段独立轨迹）
        Trajectory<TRAJ_ORDER> final_traj;
        if (!collision_gcopter_.concatenateTrajectories(segment_trajectories, final_traj)) {
            ROS_ERROR("Failed to concatenate trajectory segments");
            return;
        }

        double dtmsec = timer.toc();

        // ===== 基于 final_traj 更新碰撞事件（碰撞点/碰撞时间/前后速度） =====
        // 关键：发给控制端的 collision_point 必须来自优化后的轨迹，而不是前端固定值。
        if (use_collision_optimization.load() && has_collision_events_.load() && !collision_events_.empty() && segment_trajectories.size() >= 2) {
            const double total_duration = final_traj.getTotalDuration();
            const size_t max_boundaries = (segment_trajectories.size() >= 1) ? (segment_trajectories.size() - 1) : 0;
            const size_t collision_count = std::min(collision_events_.size(), max_boundaries);

            for (size_t ci = 0; ci < collision_count; ++ci) {
                // 碰撞时刻 = 前 ci 段持续时间之和（即第 ci 段末端）
                double tcol = 0.0;
                for (size_t s = 0; s <= ci; ++s) tcol += segment_trajectories[s].getTotalDuration();
                if (tcol < 0.0) tcol = 0.0;
                if (tcol > total_duration) tcol = total_duration;

                // 从 final_traj 采样碰撞点与碰撞前后速度
                const double eps = 1e-6;
                const double t_pre = std::max(0.0, tcol - eps);
                const double t_post = std::min(total_duration, tcol + eps);

                const Eigen::Vector3d collision_pt = final_traj.getPos(tcol);
                const Eigen::Vector3d pre_vel_from_traj = final_traj.getVel(t_pre);
                const Eigen::Vector3d post_vel_from_traj = final_traj.getVel(t_post);

                // TRT：基于优化后的 pre 段重算 post 速度
                auto &evt = collision_events_[ci];
                Eigen::Vector3d post_vel_TRT = evt.calculateDynamicPostCollisionVelocity(
                    segment_trajectories[ci], -1.0,
                    config.friction, config.damping_ratio, evt.given_yaw);

                // 写回更新，供后续发布给控制端
                evt.collision_time = tcol;
                if (collision_pt.allFinite()) {
                    evt.collision_point = collision_pt;
                }
                if (pre_vel_from_traj.allFinite()) {
                    evt.pre_collision_velocity = pre_vel_from_traj;
                }
                if (post_vel_TRT.allFinite() && post_vel_TRT.norm() > 1e-9) {
                    evt.post_collision_velocity = post_vel_TRT;
                } else if (post_vel_from_traj.allFinite()) {
                    evt.post_collision_velocity = post_vel_from_traj;
                }

                if (ci == 0) {
                    ROS_INFO("=== Updated Collision Events from final_traj ===");
                }
                ROS_INFO("  Event %zu: t=%.3f, pos=(%.3f,%.3f,%.3f)",
                         ci, evt.collision_time,
                         evt.collision_point.x(), evt.collision_point.y(), evt.collision_point.z());
            }
        }

        // 计算轨迹长度（减少采样以提高实时性）
        int segnum = std::min(10000, std::max(100, int(final_traj.getTotalDuration() * 100.0)));
        double dt = final_traj.getTotalDuration() / segnum;
        Eigen::Vector3d xk, xk1 = final_traj.getPos(0.0);
        double length = 0.0;
        for (int i = 0; i < segnum; i++) {
            xk = xk1;
            xk1 = final_traj.getPos(dt * (i + 1.0));
            length += (xk1 - xk).norm();
        }

        // 显示轨迹信息
        printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
        printf("Multi-segment trajectory optimization completed.\nTrajectory profile:\n");
        printf("       Total   computation  time: %7.2lf msecs.\n", dtmsec);
        printf("       Total   flight   duration: %7.2lf secs.\n", final_traj.getTotalDuration());
        printf("       Total  trajectory  length: %7.2lf m.\n", length);
        printf("       Average          velocity: %7.2lf m/s.\n", length / final_traj.getTotalDuration());
        printf("       Max  velocity   magnitude: %7.2lf m/s.\n", final_traj.getMaxVelRate());
        printf("       Max accleration magnitude: %7.2lf m/s^2.\n", final_traj.getMaxAccRate());
        printf("       Number of collision events: %lu\n", collision_events_.size());
        printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

        // 如果存在碰撞事件，打印优化后的碰撞前/碰撞后速度信息
        if (use_collision_optimization.load() && has_collision_events_.load() && !collision_events_.empty()) {
            size_t collision_count = collision_events_.size();
            double totalT = final_traj.getTotalDuration();
            printf("---- Collision velocities (after optimization) ----\n");
            for (size_t ci = 0; ci < collision_count && ci < (size_t)final_traj.getPieceNum(); ++ci) {
                // Prefer the authoritative, updated values stored in collision_events_
                double tcol = collision_events_[ci].collision_time;
                // Fallback: if collision_time not set or invalid, compute from final_traj piece durations
                if (!std::isfinite(tcol) || tcol < 0.0) {
                    tcol = 0.0;
                    for (int s = 0; s <= (int)ci; ++s) tcol += final_traj[s].getDuration();
                    if (tcol > totalT) tcol = totalT;
                }

                Eigen::Vector3d pre_vel = collision_events_[ci].pre_collision_velocity;
                Eigen::Vector3d post_vel = collision_events_[ci].post_collision_velocity;

                // If for some reason the stored velocities are not finite, fallback to sampling final_traj
                if (!pre_vel.allFinite()) {
                    double eps = 1e-6;
                    double tpre = std::max(0.0, tcol - eps);
                    pre_vel = final_traj.getVel(tpre);
                }
                if (!post_vel.allFinite()) {
                    double eps = 1e-6;
                    double tpost = std::min(totalT, tcol + eps);
                    post_vel = final_traj.getVel(tpost);
                }

                printf("Collision %zu at t=%.3f s: pre_vel=(%.3f, %.3f, %.3f), post_vel=(%.3f, %.3f, %.3f)\n",
                       ci, tcol,
                       pre_vel.x(), pre_vel.y(), pre_vel.z(),
                       post_vel.x(), post_vel.y(), post_vel.z());
            }
            printf("-----------------------------------------------\n\n");
        }

        if (final_traj.getPieceNum() > 0) {
            visualizer.visualize(final_traj, 0, true);
            int samples = int(length / 0.1);
            visualizer.visualizeEllipsoid(final_traj, samples);
            // 导出最终拼接轨迹为 CSV（0.01s 采样）以便验证
            try {
                std::string csv_path = resolveDataCsvPathTrajGen(std::string("final_traj.csv"));
                exportTrajectoryCSV(csv_path, final_traj, 0.01);
                ROS_INFO("Exported final_traj to %s", csv_path.c_str());
            } catch (...) {
                ROS_WARN("Failed to export final_traj.csv (exception)");
            }
            
            // 发布轨迹
            ros::Time unified_start_time = ros::Time::now() + ros::Duration(config.traj_start_delay);
            quadrotor_msgs::PolynomialTrajectory vistrajMsg;
            genPolyTrajMsg(final_traj, Eigen::Isometry3d::Identity(), unified_start_time, vistrajMsg);
            
            // 在发布前进行快速可行性检查（宽松阈值）
            double constraint_tol = 5e-2; // 5 cm
            double vel_tol = 0.05;        // 5 cm/s
            double acc_tol = 0.5;         // 0.5 m/s^2

            bool feasible = isTrajectoryFeasible(final_traj, constraint_tol, vel_tol, acc_tol);

            const double v_lim = config.maxVelRate;
            const double a_lim = config.maxAccRate;
            const double k_max = 1.2;
            const double safety = 1.0;

            bool allow_time_scaled_publish = false;
            if (feasible) {
                allow_time_scaled_publish = true;
            } else if (last_rms_violation_ <= constraint_tol && (last_v_obs_ > v_lim || last_a_obs_ > a_lim)) {
                allow_time_scaled_publish = true;
                ROS_WARN("Multi-segment: dynamics overshoot detected but RMS within tol (%.6fm); will apply single-pass time-scaling", last_rms_violation_);
            }

            if (!allow_time_scaled_publish) {
                ROS_WARN("Multi-segment optimized trajectory failed feasibility check - skipping publish");
            } else {
                // 计算缩放系数并应用到 per-segment time
                double v_obs = last_v_obs_;
                double a_obs = last_a_obs_;
                double k = 1.0;
                if (v_obs > v_lim) {
                    k = (v_obs / v_lim) * safety;
                } else if (a_obs > a_lim) {
                    k = std::sqrt(a_obs / a_lim) * safety;
                }
                if (k < 1.0) k = 1.0;
                if (k > k_max) k = k_max;

                if (k > 1.0001) {
                    ROS_WARN("Applying time-scaling k=%.3f (v_obs=%.3f a_obs=%.3f) to multi-segment trajectory", k, v_obs, a_obs);
                }

                // Apply scaling to vistrajMsg.time
                if (k > 1.0001) {
                    for (size_t ti = 0; ti < vistrajMsg.time.size(); ++ti) {
                        vistrajMsg.time[ti] = vistrajMsg.time[ti] * k;
                    }
                }

                trajMsg = vistrajMsg;
                trajPub.publish(trajMsg);
                visTrajPub.publish(trajMsg);

                // ===== 发布碰撞轨迹消息（单碰撞情况）=====
                if (use_collision_optimization.load() && has_collision_events_.load() && !collision_events_.empty() && collisionTrajPub) {
                    quadrotor_msgs::CollisionTrajectory col_traj_msg;
                    col_traj_msg.header.stamp = unified_start_time;
                    col_traj_msg.header.frame_id = "world";

                    // 构建碰撞事件消息
                    std::vector<quadrotor_msgs::CollisionEvent> collision_event_msgs;
                    for (const auto& evt : collision_events_) {
                        quadrotor_msgs::CollisionEvent evt_msg;
                        
                        // 碰撞时刻（相对于轨迹起始时间）
                        evt_msg.collision_time = evt.collision_time;
                        
                        // 碰撞点位置
                        evt_msg.collision_point.x = evt.collision_point.x();
                        evt_msg.collision_point.y = evt.collision_point.y();
                        evt_msg.collision_point.z = evt.collision_point.z();
                        
                        // 碰撞平面法向
                        evt_msg.plane_normal.x = evt.plane_normal.x();
                        evt_msg.plane_normal.y = evt.plane_normal.y();
                        evt_msg.plane_normal.z = evt.plane_normal.z();
                        
                        // 碰撞前速度（从 final_traj 重新计算）
                        evt_msg.pre_collision_velocity.x = evt.pre_collision_velocity.x();
                        evt_msg.pre_collision_velocity.y = evt.pre_collision_velocity.y();
                        evt_msg.pre_collision_velocity.z = evt.pre_collision_velocity.z();
                        
                        // 碰撞后速度（TRT 计算或从 final_traj）
                        evt_msg.post_collision_velocity.x = evt.post_collision_velocity.x();
                        evt_msg.post_collision_velocity.y = evt.post_collision_velocity.y();
                        evt_msg.post_collision_velocity.z = evt.post_collision_velocity.z();
                        
                        collision_event_msgs.push_back(evt_msg);
                    }

                    col_traj_msg.collision_events = collision_event_msgs;
                    collisionTrajPub.publish(col_traj_msg);
                    
                    ROS_INFO("Published collision trajectory with %zu events to controller", 
                             collision_event_msgs.size());
                    // mark optimization completed and published
                     trajectory_optimized_.store(true);
                    ROS_INFO("  Event: t=%.3f, pos=(%.3f,%.3f,%.3f), pre_vel=(%.3f,%.3f,%.3f), post_vel=(%.3f,%.3f,%.3f)",
                             collision_events_[0].collision_time,
                             collision_events_[0].collision_point.x(),
                             collision_events_[0].collision_point.y(),
                             collision_events_[0].collision_point.z(),
                             collision_events_[0].pre_collision_velocity.x(),
                             collision_events_[0].pre_collision_velocity.y(),
                             collision_events_[0].pre_collision_velocity.z(),
                             collision_events_[0].post_collision_velocity.x(),
                             collision_events_[0].post_collision_velocity.y(),
                             collision_events_[0].post_collision_velocity.z());
                } else if (!use_collision_optimization.load() && collisionTrajPub) {
                    quadrotor_msgs::CollisionTrajectory col_traj_msg;
                    col_traj_msg.header.stamp = unified_start_time;
                    col_traj_msg.header.frame_id = "world";
                    collisionTrajPub.publish(col_traj_msg);
                }
            }
        }
    }

};
#endif