#ifndef _INTENTION_GET_CORR_DIR_HPP_
#define _INTENTION_GET_CORR_DIR_HPP_

#include "intention_get_corridor/config.hpp"
#include "intention_get_corridor/glbmap.hpp"
#include "intention_get_corridor/visualizer.hpp"
#include "intention_get_corridor/sample_forward.hpp"
#include "intention_get_corridor/astar.hpp"
#include "intention_get_corridor/solver/firi.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <chrono>
#include <random>
#include <string>
#include <fstream>
#include <sstream>
#include <array>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <queue>
#include <deque>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <quadrotor_msgs/Corridor.h>
#include <quadrotor_msgs/CorridorList.h>
#include <quadrotor_msgs/TrajectoryPlan.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/CollisionTrajectory.h>
// #include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

class GlobalPlanner
{
private:
    Config config;

    struct PlaneCandidate;

    struct BfsDistSnapshot
    {
        int ix_min = 0, ix_max = -1;
        int iy_min = 0, iy_max = -1;
        int zIdx = 0;
        double cellS = 0.0;
        double base_z = 0.0;
        std::shared_ptr<const std::unordered_map<long long, int>> dist;
    };

    ros::NodeHandle nh;
    ros::Subscriber targetSub;
    ros::Subscriber odomSub;
    ros::Subscriber mapSub;
    ros::Subscriber boundSub;
    ros::Subscriber intentionSub;
    ros::Publisher intentionPub;
    ros::Publisher keyPosPub;
    ros::Publisher targetPub;
    ros::Publisher mapPub, visMapPub;
    ros::Publisher corridorPub;
    ros::Publisher collision_trajectory_pub_;  // 完整碰撞轨迹发布器（CollisionTrajectory）
    // subscribe to trajectory execution status from controller
    ros::Subscriber status_pred_pose_sub;
    ros::Subscriber status_pred_vel_sub;
    ros::Subscriber status_pred_acc_sub;
    ros::Subscriber status_remaining_sub;
    ros::Subscriber status_segment_sub;
    ros::Subscriber collision_trigger_sub;
    ros::Subscriber ext_force_sub_;
    // publisher for frontier plane info (text)
    ros::Publisher frontierPlanePub;

    std::atomic<bool> mapInitialized{false}, odomInitialized{false};
    // preset-related auto-trigger removed to avoid redundancy
    std::shared_ptr<GlobalMap> glbMapPtr;
    Visualizer visualizer;
    Eigen::Vector3d bound_min, bound_max;

    std::vector<Eigen::Matrix<double, 6, -1>> onetimeCorridor, partCorridor;
    geometry_msgs::PoseStamped last_target, lastmsg;
    Eigen::Matrix<double, 6, -1> lastPolytope{6, 0};
    bool getFirstTarget = false;
    int confirm = 0;

    // Worker for periodic corridor generation
    std::thread corridor_worker_;
    std::mutex corridor_mutex_;
    std::condition_variable corridor_cv_;
    std::atomic<bool> corridor_thread_stop_{false};
    std::atomic<bool> corridor_force_trigger_{false};
    std::atomic<bool> corridor_ready_{false};
    std::atomic<bool> corridor_busy_{false};
    std::atomic<uint64_t> corridor_version_{0};
    // When true, generation routines should emit INFO logs. Set true for explicit requests.
    std::atomic<bool> generation_verbose_{false};
    // mutexes to protect shared mutable inputs
    std::mutex odom_mutex_;
    std::mutex bound_mutex_;
    // recent odom positions for revisit penalty (protected by odom_mutex_)
    std::deque<std::pair<ros::Time, Eigen::Vector3d>> recent_odom_positions_;
    // Cache the last published CollisionTrajectory so publishCompleteCollisionTrajectory
    // can reuse it when RRT hasn't produced a collision path (avoid publishing empty messages)
    quadrotor_msgs::CollisionTrajectory last_collision_traj_;
    std::mutex last_collision_mutex_;
    bool has_last_collision_traj_ = false;
    std::chrono::steady_clock::time_point last_corridor_time_{};
    Eigen::Vector3d last_corridor_origin_{Eigen::Vector3d::Zero()};
    double last_corridor_yaw_ = 0.0;
    // latest corridor stored as shared_ptr for atomic swap/load (double-buffer)
    std::shared_ptr<std::vector<Eigen::Matrix<double, 6, -1>>> latest_corridor_ptr_{
        std::make_shared<std::vector<Eigen::Matrix<double, 6, -1>>>()
    };

    // latest BFS dist snapshot (paired with latest corridor generation)
    std::shared_ptr<BfsDistSnapshot> latest_bfs_ptr_{nullptr};

    // latest sampled BFS path points (world frame), used for direction fallback when verified plane is None
    std::shared_ptr<std::vector<Eigen::Vector3d>> latest_bfs_path_ptr_{nullptr};

    // A* global and local path snapshots (world frame). These replace BFS when BFS is disabled.
    std::mutex astar_mutex_;
    std::unique_ptr<astar_ros> astar_planner_{nullptr};
    std::shared_ptr<std::vector<Eigen::Vector3d>> latest_astar_global_path_ptr_{nullptr};
    std::shared_ptr<std::vector<Eigen::Vector3d>> latest_astar_local_path_ptr_{nullptr};
    std::mutex astar_cache_mutex_;
    bool astar_cached_goal_valid_{false};
    Eigen::Vector3d astar_cached_goal_{Eigen::Vector3d::Zero()};

    // A* local-crop progress guard: prevent snapping back to older segments of the cached global path.
    // Symptom it prevents: when the vehicle drifts off the path, the closest-point index can jump
    // backward, causing oscillation (back-and-forth) when repeatedly cropping the global path.
    std::mutex astar_progress_mutex_;
    bool astar_progress_valid_{false};
    size_t astar_last_progress_idx_{0};
    double astar_backtrack_allow_m_{2.0};

    const double corridor_pos_thresh_ = 0.05;           // meters
    const double corridor_yaw_thresh_ = 3.0 * M_PI/180.0; // radians
    const std::chrono::milliseconds corridor_period_{500}; // 500 ms

    nav_msgs::Odometry odom;

    // latest execution status from controller (protected by mutex)
    std::mutex status_mutex_;
    geometry_msgs::PoseStamped latest_pred_pose;
    geometry_msgs::Vector3 latest_pred_vel;
    geometry_msgs::Vector3 latest_pred_acc;
    double latest_remaining_time = 0.0;
    int latest_segment_idx = -1;

    // collision trigger from controller (gate for replan trigger)
    std::atomic<bool> collision_trigger_{false};
    std::string collision_trigger_topic_{"/collision_trigger"};
    bool collision_trigger_default_{false};
    bool use_time_based_collision_end_trigger_{true};
    double collision_external_force_threshold_ = 4.0;
    std::atomic<bool> is_force_high_{false};
    int consecutive_low_force_count_{0};
    // Previous planned-collision-end trigger state.
    bool prev_collision_trigger_state_{false};
    int consecutive_collision_false_count_{0};
    int collision_false_confirm_count_{3}; // reused by force debounce logic

    // Parallel to latest_remaining_time: event-driven replan request (latched).
    // Set by periodic kinematics check; cleared after a successful publish.
    std::atomic<bool> collision_event_trigger_{false};

    // 是否“正在执行碰撞轨迹”（以本节点发布过 MODE_COLLISION 作为执行态的近似判据）。
    // bridge 当前发布的是“计划碰撞结束”信号，因此这里不再直接与 /collision_trigger 同步。
    std::atomic<bool> executing_collision_traj_{false};

    // 执行态保持窗口（秒）：发布 collision 轨迹后，至少在该窗口内认为仍在执行碰撞轨迹。
    // 目的：外力触发的 /collision_trigger 可能很快从 true 变回 false，但控制器仍在执行碰撞轨迹。
    double collision_exec_hold_sec_{1.0};
    std::atomic<double> collision_exec_until_sec_{0.0};

    // 在边沿触发（碰撞结束）之后，抑制剩余时间触发一段时间，防止同一时刻重复重规划。
    std::atomic<double> suppress_time_replan_until_sec_{0.0};

    // 防止“剩余时间触发 + 边沿触发”同时派发多个异步重规划线程。
    std::atomic<bool> replan_dispatch_inflight_{false};

    // 冷却：分别用于“剩余时间触发”和“边沿触发(碰撞结束)”
    ros::Time last_time_trigger_replan_time_{ros::Time(0)};
    ros::Time last_edge_trigger_replan_time_{ros::Time(0)};
    // Latch: once mixed planning outputs a collision trajectory candidate, suppress
    // remaining-time and edge-triggered replans until collision-specific handling progresses.
    std::atomic<bool> collision_candidate_active_{false};
    // Time of last collision-trigger-driven publish that should suppress immediate duplicates
    ros::Time last_collision_trigger_time_{ros::Time(0)};
    // Suppression duration (seconds) to make time-edge and event triggers mutually exclusive briefly
    double collision_trigger_suppress_s_ = 0.1;
    const double collision_event_normal_dist_thresh_m_{3.0};
    // goal / replan suppression state
    Eigen::Vector3d latest_goal_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d current_traj_goal_{Eigen::Vector3d::Zero()};
    double arrival_remaining_time_thresh_{0.3};
    enum class SpecialSceneReplanStage : int
    {
        Disabled = 0,
        AwaitInitialCollisionPlan = 1,
        AwaitCollisionEnd = 2,
        AwaitFinalAvoidance = 3,
        Completed = 4,
    };
    std::atomic<int> special_scene_stage_{static_cast<int>(SpecialSceneReplanStage::Disabled)};
    double special_scene_post_collision_wait_sec_{0.5};
    double special_scene_final_astar_cruise_speed_mps_{1.2};
    double special_scene_final_astar_min_dt_{0.25};
    double special_scene_final_astar_waypoint_spacing_m_{0.8};
    // emergency trigger debounce
    const int emergency_cooldown_ms = 1000;
    // for debouncing frontier publications
    std::string last_frontier_text_;
    std::mutex frontier_pub_mutex_;

    // selected verified frontier plane (based on BFS graph distance); empty if none
    std::mutex selected_frontier_plane_mutex_;
    bool has_selected_frontier_plane_ = false;
    Eigen::Vector3d selected_frontier_plane_normal_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d selected_frontier_plane_point_ = Eigen::Vector3d::Zero();
    double selected_frontier_plane_width_ = 0.0;
    int selected_frontier_plane_corridor_idx_ = -1;
    int selected_frontier_plane_face_idx_ = -1;
    int selected_frontier_plane_bfs_steps_ = -1;

    // selected collision plane among unverified vertical candidates
    std::mutex selected_collision_plane_mutex_;
    bool has_selected_collision_plane_ = false;
    Eigen::Vector3d selected_collision_plane_normal_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d selected_collision_plane_point_ = Eigen::Vector3d::Zero();
    double selected_collision_plane_width_ = 0.0;
    double selected_collision_plane_distance_m_ = 0.0;
    int selected_collision_plane_corridor_idx_ = -1;
    int selected_collision_plane_face_idx_ = -1;

    struct PlaneCandidate
    {
        int corridor_idx = -1;
        int face_idx = -1;
        bool vertical = false;
        bool verified_frontier = false;
        Eigen::Vector3d normal_unit = Eigen::Vector3d::Zero();
        Eigen::Vector3d point_on_plane = Eigen::Vector3d::Zero();
        double d = 0.0;      // plane: n·x + d = 0 (unit n)
        double width = 0.0;  // tangential span (meters) within the polytope face
    };

    inline std::vector<PlaneCandidate> extractPlaneCandidatesFromCorridorSeq(const std::vector<Eigen::Matrix<double, 6, -1>> &corridorSeq)
    {
        std::vector<PlaneCandidate> candidates;
        candidates.reserve(corridorSeq.size() * 12);

        int global_cor_idx = 0;
        for (auto iter = corridorSeq.begin(); iter != corridorSeq.end(); ++iter)
        {
            // Precompute polytope vertices once (for face width computation)
            std::vector<Eigen::Vector3d> poly_vertices;
            {
                const int m = iter->cols();
                std::vector<Eigen::Vector3d> normals(m);
                std::vector<double> bs(m);
                for (int pi = 0; pi < m; ++pi)
                {
                    const Eigen::Vector3d n = iter->col(pi).head<3>();
                    const Eigen::Vector3d p = iter->col(pi).tail<3>();
                    normals[pi] = n;
                    bs[pi] = n.dot(p);
                }

                const double det_eps = 1e-9;
                const double inside_tol = 1e-6;
                const double dedup_tol = 1e-3;
                for (int a = 0; a < m; ++a)
                {
                    for (int b = a + 1; b < m; ++b)
                    {
                        for (int c = b + 1; c < m; ++c)
                        {
                            Eigen::Matrix3d A;
                            A.row(0) = normals[a].transpose();
                            A.row(1) = normals[b].transpose();
                            A.row(2) = normals[c].transpose();
                            const double det = A.determinant();
                            if (!std::isfinite(det) || std::fabs(det) < det_eps)
                                continue;
                            const Eigen::Vector3d rhs(bs[a], bs[b], bs[c]);
                            const Eigen::Vector3d x = A.fullPivLu().solve(rhs);
                            if (!x.allFinite())
                                continue;

                            bool inside = true;
                            for (int k = 0; k < m; ++k)
                            {
                                const double v = normals[k].dot(x);
                                if (!std::isfinite(v) || v > bs[k] + inside_tol)
                                {
                                    inside = false;
                                    break;
                                }
                            }
                            if (!inside)
                                continue;

                            bool is_new = true;
                            for (const auto &vtx : poly_vertices)
                            {
                                if ((vtx - x).norm() < dedup_tol)
                                {
                                    is_new = false;
                                    break;
                                }
                            }
                            if (is_new)
                                poly_vertices.push_back(x);
                        }
                    }
                }
            }

            for (int i = 0; i < iter->cols(); i++)
            {
                const Eigen::Vector3d n_raw = iter->col(i).head<3>();
                const Eigen::Vector3d p_raw = iter->col(i).tail<3>();

                const double nz = n_raw.z();
                const double vertical_thresh = 0.5; // if |nz| < thresh consider vertical wall
                if (std::fabs(nz) >= vertical_thresh)
                    continue;

                double nx = n_raw.x(), ny = n_raw.y(), nnz = n_raw.z();
                const double nrm = std::sqrt(nx * nx + ny * ny + nnz * nnz);
                if (nrm <= 1e-6)
                    continue;
                nx /= nrm;
                ny /= nrm;
                nnz /= nrm;

                PlaneCandidate cand;
                cand.corridor_idx = global_cor_idx;
                cand.face_idx = i;
                cand.vertical = true;
                cand.verified_frontier = false;
                cand.normal_unit = Eigen::Vector3d(nx, ny, nnz);
                // compute face centroid from poly_vertices (fallback to p_raw)
                {
                    Eigen::Vector3d face_centroid = p_raw;
                    if (!poly_vertices.empty()) {
                        const double b_plane = cand.normal_unit.dot(p_raw);
                        const double face_tol = 5e-3;
                        std::vector<Eigen::Vector3d> face_pts;
                        face_pts.reserve(poly_vertices.size());
                        for (const auto &vtx : poly_vertices) {
                            const double dist_plane = std::fabs(cand.normal_unit.dot(vtx) - b_plane);
                            if (dist_plane <= face_tol) face_pts.push_back(vtx);
                        }
                        if (!face_pts.empty()) {
                            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
                            for (const auto &v : face_pts) mean += v;
                            mean /= static_cast<double>(face_pts.size());
                            // project mean back onto the plane to remove numeric drift
                            const double diff = cand.normal_unit.dot(mean) - b_plane;
                            mean -= diff * cand.normal_unit;
                            face_centroid = mean;
                        }
                    }
                    cand.point_on_plane = face_centroid;
                }
                cand.d = -(cand.normal_unit.dot(cand.point_on_plane));

                // width along a horizontal tangent inside the plane
                {
                    const Eigen::Vector3d world_z(0.0, 0.0, 1.0);
                    Eigen::Vector3d t = world_z.cross(cand.normal_unit);
                    if (t.norm() < 1e-6)
                    {
                        const Eigen::Vector3d world_x(1.0, 0.0, 0.0);
                        t = world_x.cross(cand.normal_unit);
                    }
                    const double tn = t.norm();
                    if (tn > 1e-9 && !poly_vertices.empty())
                    {
                        t /= tn;
                        const double b_plane = cand.normal_unit.dot(cand.point_on_plane);
                        const double face_tol = 5e-3;
                        double smin = 1e100, smax = -1e100;
                        int cnt_face = 0;
                        for (const auto &vtx : poly_vertices)
                        {
                            const double dist_plane = std::fabs(cand.normal_unit.dot(vtx) - b_plane);
                            if (dist_plane <= face_tol)
                            {
                                const double s = t.dot(vtx);
                                if (s < smin)
                                    smin = s;
                                if (s > smax)
                                    smax = s;
                                ++cnt_face;
                            }
                        }
                        if (cnt_face >= 2 && std::isfinite(smin) && std::isfinite(smax) && smax >= smin)
                        {
                            cand.width = smax - smin;
                        }
                        else
                        {
                            cand.width = 0.0;
                        }
                    }
                    else
                    {
                        cand.width = 0.0;
                    }
                    }

                candidates.push_back(cand);
            }

            ++global_cor_idx;
        }

        return candidates;
    }

    // Mark candidates as verified frontier by sampling outside along plane normal using safeQuery.
    inline void markVerifiedFrontiersBySafeQuery(std::vector<PlaneCandidate> &candidates)
    {
        if (!glbMapPtr || !glbMapPtr->ogmPtr) return;
        auto ogm = glbMapPtr->ogmPtr;
        const double base = std::max(0.05, ogm->getScale() * 0.5);
        const double offsets[3] = {0.0, 0.10, 0.20};

        for (auto &c : candidates)
        {
            // Only vertical faces are considered for frontier verification (same rule as pubCorridor)
            if (!c.vertical)
            {
                c.verified_frontier = false;
                continue;
            }

            bool all_free = true;
            for (int oi = 0; oi < 3; ++oi)
            {
                Eigen::Vector3d sample_p = c.point_on_plane + (base + offsets[oi]) * c.normal_unit;
                bool free = false;
                try { free = glbMapPtr->safeQuery(sample_p); } catch(...) { free = false; }
                if (!free) { all_free = false; break; }
            }
            c.verified_frontier = all_free;
        }
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
                if (incorridor(full_corridor[i], pt))
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
    // preset auto-trigger logic removed

public:
    // Public wrapper for map initialization (calls private implementation)
    inline void initializeMap()
    {
        initializeMapImpl();
    }
    GlobalPlanner(Config &conf, ros::NodeHandle &nh_)
            : config(conf), nh(nh_), mapInitialized(false), odomInitialized(false),
                glbMapPtr(std::make_shared<GlobalMap>(config)),
                visualizer(config, nh)
    {
        mapPub = nh.advertise<sensor_msgs::PointCloud2>(config.infmapTopic, 1000);
        visMapPub = nh.advertise<sensor_msgs::PointCloud2>(config.infmapTopic + "_vis", 1000);
        corridorPub = nh.advertise<quadrotor_msgs::CorridorList>("corridor_list", 1000);
        frontierPlanePub = nh.advertise<std_msgs::String>("frontier_planes", 1000);
        keyPosPub = nh.advertise<quadrotor_msgs::TrajectoryPlan>("key_pos", 1000);
        collision_trajectory_pub_ = nh.advertise<quadrotor_msgs::CollisionTrajectory>("complete_collision_trajectory", 1000);  // 发布 CollisionTrajectory
        targetPub = nh.advertise<geometry_msgs::PoseStamped>(config.targetTopic, 1);
        mapSub = nh.subscribe("/globalmap", 1,
                              &GlobalPlanner::MapCallback, this,
                              ros::TransportHints().tcpNoDelay());
        boundSub = nh.subscribe("/boundmap", 1,
                                &GlobalPlanner::BoundCallback, this,
                                ros::TransportHints().tcpNoDelay());
        odomSub = nh.subscribe(config.odomTopic, 1,
                               &GlobalPlanner::OdomCallback, this,
                               ros::TransportHints().tcpNoDelay());

        // subscribe to controller status publications
        status_pred_pose_sub = nh.subscribe("trajectory_predicted_pose", 1,
                            &GlobalPlanner::statusPredPoseCallback, this,
                            ros::TransportHints().tcpNoDelay());
        status_pred_vel_sub = nh.subscribe("trajectory_predicted_velocity", 1,
                           &GlobalPlanner::statusPredVelCallback, this,
                           ros::TransportHints().tcpNoDelay());
        status_pred_acc_sub = nh.subscribe("trajectory_predicted_acceleration", 1,
                           &GlobalPlanner::statusPredAccCallback, this,
                           ros::TransportHints().tcpNoDelay());
        status_remaining_sub = nh.subscribe("trajectory_remaining_time", 1,
                            &GlobalPlanner::statusRemainingCallback, this,
                            ros::TransportHints().tcpNoDelay());
        status_segment_sub = nh.subscribe("trajectory_current_segment", 1,
                          &GlobalPlanner::statusSegmentCallback, this,
                          ros::TransportHints().tcpNoDelay());

        // collision trigger gate (published by controller)
        nh.param<std::string>("collision_trigger_topic", collision_trigger_topic_, collision_trigger_topic_);
        nh.param("collision_trigger_default", collision_trigger_default_, collision_trigger_default_);
        nh.param("use_time_based_collision_end_trigger", use_time_based_collision_end_trigger_,
                 use_time_based_collision_end_trigger_);
        nh.param("collision_false_confirm_count", collision_false_confirm_count_, collision_false_confirm_count_);
        nh.param("collision_exec_hold_sec", collision_exec_hold_sec_, collision_exec_hold_sec_);
        nh.param("CollisionModel/collision_force_threshold", collision_external_force_threshold_, collision_external_force_threshold_);
        collision_trigger_.store(collision_trigger_default_);
        collision_trigger_sub = nh.subscribe(collision_trigger_topic_, 1,
                    &GlobalPlanner::collisionTriggerCallback, this,
                    ros::TransportHints().tcpNoDelay());
        ext_force_sub_ = nh.subscribe("/external_force_est", 1,
                    &GlobalPlanner::extForceCallback, this,
                    ros::TransportHints().tcpNoDelay());

        if (config.useLoadPCDFile)
        {
            targetSub = nh.subscribe(config.targetTopic, 1,
                                        &GlobalPlanner::setposCallBack, this,
                                        ros::TransportHints().tcpNoDelay());
            intentionPub = nh.advertise<quadrotor_msgs::TrajectoryPlan>("/MyPointSeq", 1000);
        }
        else
        {
            // Also subscribe to targetTopic so RViz /goal or external SetPos triggers setposCallBack
            targetSub = nh.subscribe(config.targetTopic, 1,
                                    &GlobalPlanner::setposCallBack, this,
                                    ros::TransportHints().tcpNoDelay());

            // 新增这一行：即使是非 PCD 模式，也把 intentionPub 初始化好
            intentionPub = nh.advertise<quadrotor_msgs::TrajectoryPlan>("/MyPointSeq", 1000);
        }
        // NOTE: The periodic corridor worker is disabled. We build the full corridor once
        // after global A* is available, then select/publish subsets on demand.

        // A* planner (used for global path search + local corridor generation when BFS is disabled)
        astar_planner_ = std::make_unique<astar_ros>(nh, config);

        nh.param("special_scene_post_collision_wait_sec",
                 special_scene_post_collision_wait_sec_,
                 special_scene_post_collision_wait_sec_);
        special_scene_stage_.store(static_cast<int>(SpecialSceneReplanStage::Disabled));
        ROS_WARN("Two-stage fixed-scene replan mode enabled for scene_real_model.xml");
    }

    ~GlobalPlanner()
    {
        // Worker thread not started.
    }

    inline bool isExecutingCollisionTrajectoryNow() const
    {
        const double now_sec = ros::Time::now().toSec();
        return executing_collision_traj_.load() || (now_sec < collision_exec_until_sec_.load());
    }

    inline void dispatchAsyncReplanOnce(const char *reason)
    {
        if (!reason)
            reason = "unknown";

        // Only allow one inflight replan thread at a time.
        if (replan_dispatch_inflight_.exchange(true))
            return;

        std::thread([this, reason]() {
            try
            {
                quadrotor_msgs::TrajectoryPlan tmp_plan;
                Eigen::Matrix3Xd dummy_keypos;
                this->getCorridorFromSetPos(tmp_plan, dummy_keypos);
            }
            catch (const std::exception &e)
            {
                ROS_WARN("Exception in getCorridorFromSetPos (%s): %s", reason, e.what());
            }
            catch (...)
            {
                ROS_WARN("Unknown exception in getCorridorFromSetPos (%s)", reason);
            }
            replan_dispatch_inflight_.store(false);
        }).detach();
    }

    inline SpecialSceneReplanStage getSpecialSceneStage() const
    {
        return static_cast<SpecialSceneReplanStage>(special_scene_stage_.load());
    }

    inline void resetSpecialSceneStateForNewGoal()
    {
        special_scene_stage_.store(static_cast<int>(SpecialSceneReplanStage::AwaitInitialCollisionPlan));
        collision_event_trigger_.store(false);
        collision_candidate_active_.store(false);
        executing_collision_traj_.store(false);
        collision_exec_until_sec_.store(0.0);
        suppress_time_replan_until_sec_.store(0.0);
    }

    inline bool handleSpecialSceneCollisionEndTrigger(const char *reason, double wait_sec)
    {
        if (getSpecialSceneStage() != SpecialSceneReplanStage::AwaitCollisionEnd)
            return false;

        collision_candidate_active_.store(false);
        collision_event_trigger_.store(true);
        executing_collision_traj_.store(false);
        collision_exec_until_sec_.store(0.0);
        special_scene_stage_.store(static_cast<int>(SpecialSceneReplanStage::AwaitFinalAvoidance));

        const ros::Time now = ros::Time::now();
        const double since = (now - last_edge_trigger_replan_time_).toSec();
        if (since <= 0.5)
            return true;

        last_edge_trigger_replan_time_ = now;
        suppress_time_replan_until_sec_.store(now.toSec() + std::max(0.2, wait_sec));
        std::thread([this, wait_sec, reason]() {
            std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.0, wait_sec)));
            try { this->dispatchAsyncReplanOnce(reason); } catch (...) { }
        }).detach();
        return true;
    }

    inline std::vector<Eigen::Vector3d> buildSpecialSceneFinalAstarPath(const std::vector<Eigen::Vector3d> &global_path,
                                                                        const Eigen::Vector3d &start,
                                                                        const Eigen::Vector3d &goal) const
    {
        std::vector<Eigen::Vector3d> raw;
        raw.reserve(global_path.size() + 2);
        raw.push_back(start);

        if (!global_path.empty())
        {
            size_t nearest_idx = 0;
            double best_d2 = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < global_path.size(); ++i)
            {
                const double d2 = (global_path[i] - start).squaredNorm();
                if (d2 < best_d2)
                {
                    best_d2 = d2;
                    nearest_idx = i;
                }
            }

            for (size_t i = nearest_idx; i < global_path.size(); ++i)
            {
                Eigen::Vector3d p = global_path[i];
                p.z() = goal.z();
                if ((p - raw.back()).norm() > 1e-3)
                {
                    raw.push_back(p);
                }
            }
        }

        if ((raw.back() - goal).norm() > 1e-3)
        {
            raw.push_back(goal);
        }
        else
        {
            raw.back() = goal;
        }

        std::vector<Eigen::Vector3d> sparse;
        sparse.reserve(raw.size());
        sparse.push_back(raw.front());
        double acc_dist = 0.0;
        const double keep_spacing = std::max(0.2, special_scene_final_astar_waypoint_spacing_m_);
        for (size_t i = 1; i + 1 < raw.size(); ++i)
        {
            acc_dist += (raw[i] - raw[i - 1]).norm();
            if (acc_dist >= keep_spacing)
            {
                sparse.push_back(raw[i]);
                acc_dist = 0.0;
            }
        }
        if ((sparse.back() - raw.back()).norm() > 1e-3)
        {
            sparse.push_back(raw.back());
        }
        else
        {
            sparse.back() = raw.back();
        }

        if (sparse.size() < 2)
        {
            sparse.clear();
            sparse.push_back(start);
            sparse.push_back(goal);
        }
        return sparse;
    }

    inline quadrotor_msgs::TrajectoryPlan buildSpecialSceneFinalAstarPlan(const std::vector<Eigen::Vector3d> &global_path,
                                                                          const Eigen::Vector3d &start,
                                                                          const Eigen::Vector3d &goal,
                                                                          const Eigen::Vector3d &dir_unit) const
    {
        const auto pts = buildSpecialSceneFinalAstarPath(global_path, start, goal);
        std::vector<double> ts;
        ts.reserve(pts.size());
        ts.push_back(0.0);
        const double cruise = std::max(0.1, special_scene_final_astar_cruise_speed_mps_);
        const double min_dt = std::max(1e-3, special_scene_final_astar_min_dt_);
        for (size_t i = 1; i < pts.size(); ++i)
        {
            const double seg_len = (pts[i] - pts[i - 1]).norm();
            const double dt = std::max(min_dt, seg_len / cruise);
            ts.push_back(ts.back() + dt);
        }
        return sample_forward::buildWaypointsPlanFromSamples(
            quadrotor_msgs::TrajectoryPlan::MODE_COLLISION_FREE, pts, ts, dir_unit);
    }

private:
    inline bool computeGlobalAstarPath(const Eigen::Vector3d &start,
                                       const Eigen::Vector3d &goal,
                                       const Eigen::Vector3d &bound_min_local,
                                       const Eigen::Vector3d &bound_max_local,
                                       std::vector<Eigen::Vector3d> &out_path)
    {
        out_path.clear();
        if (!mapInitialized.load() || !glbMapPtr)
            return false;
        if (!astar_planner_)
            return false;

        pcl::PointCloud<pcl::PointXYZ> infcloud;
        try {
            glbMapPtr->getPointCloud(infcloud, config.expectedHeight[1]);
        } catch (...) {
            return false;
        }

        std::lock_guard<std::mutex> lk(astar_mutex_);
        astar_planner_->setCloudMap(infcloud);
        astar_planner_->change_map_bd(bound_max_local, bound_min_local);
        astar_planner_->setstart(start);
        astar_planner_->setgoal(goal);
        astar_planner_->getpathlist(out_path);
        if (out_path.size() < 2)
            return false;

        // Ensure order is roughly start -> goal
        if ((out_path.front() - start).norm() > (out_path.back() - start).norm())
            std::reverse(out_path.begin(), out_path.end());

        return out_path.size() >= 2;
    }

    // Build global A* path and FULL corridor ONCE (static map assumption), cached by goal.
    // This replaces the periodic corridor worker.
    inline bool ensureGlobalAstarPathAndFullCorridorBuilt()
    {
        if (!odomInitialized.load() || !mapInitialized.load() || !glbMapPtr)
            return false;

        // Snapshot shared inputs
        nav_msgs::Odometry odom_local;
        Eigen::Vector3d bound_min_local, bound_max_local;
        Eigen::Vector3d goal_local;
        {
            std::lock(odom_mutex_, bound_mutex_, status_mutex_);
            std::lock_guard<std::mutex> lk1(odom_mutex_, std::adopt_lock);
            std::lock_guard<std::mutex> lk2(bound_mutex_, std::adopt_lock);
            std::lock_guard<std::mutex> lk3(status_mutex_, std::adopt_lock);
            odom_local = odom;
            bound_min_local = bound_min;
            bound_max_local = bound_max;
            goal_local = latest_goal_;
        }

        auto global_ptr = std::atomic_load(&latest_astar_global_path_ptr_);
        auto corridor_ptr = std::atomic_load(&latest_corridor_ptr_);

        bool need_recompute = true;
        {
            std::lock_guard<std::mutex> lk(astar_cache_mutex_);
            need_recompute = !global_ptr || global_ptr->size() < 2 ||
                             !corridor_ptr || corridor_ptr->empty() ||
                             !goalMatchesCachedAstarGoal(goal_local);
        }
        if (!need_recompute)
        {
            corridor_ready_.store(true);
            return true;
        }

        // Compute global A* once using current odom as start snapshot.
        Eigen::Vector3d start(odom_local.pose.pose.position.x,
                              odom_local.pose.pose.position.y,
                              odom_local.pose.pose.position.z);
        start.z() = goal_local.z();
        Eigen::Vector3d goal = goal_local;

        std::vector<Eigen::Vector3d> global_path_new;
        if (!computeGlobalAstarPath(start, goal, bound_min_local, bound_max_local, global_path_new))
        {
            ROS_WARN("ensureGlobalAstarPathAndFullCorridorBuilt: global A* failed");
            return false;
        }
        for (auto &p : global_path_new) p.z() = goal_local.z();

        // Generate FULL corridor along the global path.
        std::vector<Eigen::Matrix<double, 6, -1>> full_corridor;
        if (!corridorSeqGen(global_path_new, std::vector<Eigen::Matrix<double, 6, -1>>(), full_corridor))
        {
            ROS_WARN("ensureGlobalAstarPathAndFullCorridorBuilt: corridorSeqGen failed for global path");
            return false;
        }

        std::atomic_store(&latest_astar_global_path_ptr_, std::make_shared<std::vector<Eigen::Vector3d>>(global_path_new));
        std::atomic_store(&latest_corridor_ptr_, std::make_shared<std::vector<Eigen::Matrix<double, 6, -1>>>(std::move(full_corridor)));

        {
            std::lock_guard<std::mutex> lk(astar_cache_mutex_);
            astar_cached_goal_ = goal_local;
            astar_cached_goal_valid_ = true;
        }

        corridor_ready_.store(true);
        corridor_version_.fetch_add(1);

        // New global path => reset progress so local cropping doesn't inherit stale indices.
        resetAstarProgress();
        return true;
    }

    inline void resetAstarProgress()
    {
        std::lock_guard<std::mutex> lk(astar_progress_mutex_);
        astar_progress_valid_ = false;
        astar_last_progress_idx_ = 0;
    }

    inline bool goalMatchesCachedAstarGoal(const Eigen::Vector3d &goal) const
    {
        const double eps = std::max(0.05, this->config.gridResolution);
        return astar_cached_goal_valid_ && ((goal - astar_cached_goal_).norm() <= eps);
    }

    inline std::vector<Eigen::Vector3d> makeLocalAstarPathInAabb(const std::vector<Eigen::Vector3d> &global_path,
                                                                 const Eigen::Vector3d &base,
                                                                 const nav_msgs::Odometry &odom_local,
                                                                 const Eigen::Vector3d &bound_min_local,
                                                                 const Eigen::Vector3d &bound_max_local)
    {
        std::vector<Eigen::Vector3d> local;
        if (global_path.size() < 2)
            return local;

        // Local path crop is done inside a local box centered at current base:
        // front/back/left/right are all 8m, and the box orientation follows velocity direction.
        // IMPORTANT: This is NOT a world-axis AABB; it's a velocity-frame box.
        const double range_fb = 8.0;
        const double range_lr = 8.0;

        // Determine heading by XY velocity; fallback to yaw when speed is tiny.
        double vx = 0.0, vy = 0.0;
        try { vx = odom_local.twist.twist.linear.x; vy = odom_local.twist.twist.linear.y; } catch(...) { vx = 0.0; vy = 0.0; }
        const double speed_thresh = 0.05;
        double yaw_dir = 0.0;
        if (std::hypot(vx, vy) > speed_thresh)
        {
            yaw_dir = std::atan2(vy, vx);
        }
        else
        {
            const auto &q = odom_local.pose.pose.orientation;
            yaw_dir = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        }

        const double cA = std::cos(yaw_dir);
        const double sA = std::sin(yaw_dir);
        const Eigen::Vector2d ux(cA, sA);
        const Eigen::Vector2d uy(-sA, cA);

        // Clamp helper (world bound)
        auto inWorldBounds = [&](const Eigen::Vector3d &p)->bool {
            return p.x() >= bound_min_local.x() && p.x() <= bound_max_local.x() &&
                   p.y() >= bound_min_local.y() && p.y() <= bound_max_local.y();
        };

        auto inLocalBox = [&](const Eigen::Vector3d &p)->bool {
            if (!inWorldBounds(p)) return false;
            const Eigen::Vector2d dp(p.x() - base.x(), p.y() - base.y());
            const double f = dp.dot(ux);
            const double l = dp.dot(uy);
            return (f >= -range_fb && f <= range_fb && l >= -range_lr && l <= range_lr);
        };

        // start from the point closest to base, then take consecutive points within AABB until length cap.
        // IMPORTANT: we guard against jumping backwards along the cached global path to avoid oscillation.
        size_t lower_bound_idx = 0;
        {
            size_t last_idx_snapshot = 0;
            bool progress_valid_snapshot = false;
            double allow_backtrack_m_snapshot = 0.0;
            {
                std::lock_guard<std::mutex> lk(astar_progress_mutex_);
                last_idx_snapshot = astar_last_progress_idx_;
                progress_valid_snapshot = astar_progress_valid_;
                allow_backtrack_m_snapshot = astar_backtrack_allow_m_;
            }

            if (progress_valid_snapshot && global_path.size() >= 2)
            {
                last_idx_snapshot = std::min(last_idx_snapshot, global_path.size() - 1);
                size_t min_idx = last_idx_snapshot;
                double acc_m = 0.0;
                const double allow_m = std::max(0.0, allow_backtrack_m_snapshot);
                while (min_idx > 0 && acc_m < allow_m)
                {
                    const Eigen::Vector2d a = global_path[min_idx].head<2>();
                    const Eigen::Vector2d b = global_path[min_idx - 1].head<2>();
                    acc_m += (a - b).norm();
                    --min_idx;
                }
                lower_bound_idx = min_idx;
            }
        }

        size_t start_idx = std::min(lower_bound_idx, global_path.size() - 1);
        double best_d = std::numeric_limits<double>::infinity();
        for (size_t i = lower_bound_idx; i < global_path.size(); ++i)
        {
            const double d = (global_path[i] - base).squaredNorm();
            if (d < best_d) { best_d = d; start_idx = i; }
        }

        {
            std::lock_guard<std::mutex> lk(astar_progress_mutex_);
            if (!astar_progress_valid_)
            {
                astar_progress_valid_ = true;
                astar_last_progress_idx_ = start_idx;
            }
            else
            {
                if (astar_last_progress_idx_ >= global_path.size())
                    astar_last_progress_idx_ = start_idx;
                else
                    astar_last_progress_idx_ = std::max(astar_last_progress_idx_, start_idx);
            }
        }

        // Graph distance limit (NOT arc length): <= 13m in grid steps.
        auto ogm = (glbMapPtr && glbMapPtr->ogmPtr) ? glbMapPtr->ogmPtr : nullptr;
        double cellS = this->config.gridResolution;
        if (ogm)
        {
            const double s = ogm->getScale();
            if (s > 1e-9) cellS = s;
        }
        if (cellS <= 1e-9) cellS = 0.2;
        const double max_graph_m = 8.0;
        const int max_graph_steps = std::max(1, (int)std::floor(max_graph_m / cellS + 1e-9));
        int graph_steps = 0;

        // NOTE: The user要求的“图距离”是网格图距离（格子步数），而不是点间弧长/欧氏距离。
        // A* 可能会输出很密的连续点（0.1m 甚至更小）。如果用连续距离累加，会导致采样/截断失真。
        // 所以这里使用 OGM 的 convertPosD2I，把点映射到整数网格 index，再用 Chebyshev 计算步数。
        auto segStepsChebyshevIdx = [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b)->int {
            if (ogm)
            {
                const Eigen::Vector3i ia = ogm->convertPosD2I(Eigen::Vector3d(a.x(), a.y(), latest_goal_.z()));
                const Eigen::Vector3i ib = ogm->convertPosD2I(Eigen::Vector3d(b.x(), b.y(), latest_goal_.z()));
                const int dx = std::abs(ib.x() - ia.x());
                const int dy = std::abs(ib.y() - ia.y());
                return std::max(dx, dy);
            }

            // Fallback (no OGM): approximate by cellS-quantized Chebyshev distance.
            const double dx = std::fabs(b.x() - a.x());
            const double dy = std::fabs(b.y() - a.y());
            const double m = std::max(dx, dy);
            if (cellS <= 1e-9) return 0;
            return std::max(0, (int)std::floor(m / cellS + 1e-9));
        };

        bool started = inLocalBox(global_path[start_idx]);
        if (started)
        {
            Eigen::Vector3d p = global_path[start_idx];
            p.z() = latest_goal_.z();
            local.push_back(p);
        }

        const double sample_graph_m = 0.8;
        const int sample_steps = std::max(1, (int)std::ceil(sample_graph_m / cellS - 1e-9));
        int steps_since_last_sample = 0;

        if (generation_verbose_.load())
        {
            ROS_INFO("makeLocalAstarPathInAabb: cellS=%.3f max_steps=%d sample_steps=%d start_idx=%zu started=%d", cellS, max_graph_steps, sample_steps, start_idx, (int)started);
        }

        for (size_t i = start_idx + 1; i < global_path.size(); ++i)
        {
            const Eigen::Vector3d p1 = global_path[i];

            if (!started)
            {
                if (inLocalBox(p1))
                {
                    started = true;
                    Eigen::Vector3d p = p1;
                    p.z() = latest_goal_.z();
                    local.push_back(p);
                    steps_since_last_sample = 0;
                }
                continue;
            }

            if (!inLocalBox(p1))
            {
                if (generation_verbose_.load())
                {
                    const Eigen::Vector2d dp(p1.x() - base.x(), p1.y() - base.y());
                    const double f = dp.dot(ux);
                    const double l = dp.dot(uy);
                    ROS_INFO("makeLocalAstarPathInAabb: stop by leaving box at i=%zu p=(%.3f,%.3f) f=%.3f l=%.3f graph_steps=%d local_sz=%zu", i, p1.x(), p1.y(), f, l, graph_steps, local.size());
                }
                // Optionally add the last point just before exiting the box if not recently added
                if (steps_since_last_sample > 0) {
                    Eigen::Vector3d p = global_path[i - 1];
                    p.z() = latest_goal_.z();
                    if (local.empty() || (p.head<2>() - local.back().head<2>()).norm() > 1e-6)
                        local.push_back(p);
                }
                break;
            }

            Eigen::Vector3d p = p1;
            p.z() = latest_goal_.z();

            const int seg_steps = segStepsChebyshevIdx(global_path[i - 1], p1);
            if (graph_steps + seg_steps > max_graph_steps)
            {
                if (generation_verbose_.load())
                {
                    ROS_INFO("makeLocalAstarPathInAabb: stop by graph limit at i=%zu seg_steps=%d graph_steps=%d/%d local_sz=%zu", i, seg_steps, graph_steps, max_graph_steps, local.size());
                }
                if (steps_since_last_sample > 0) {
                    Eigen::Vector3d p_prev = global_path[i - 1];
                    p_prev.z() = latest_goal_.z();
                    if (local.empty() || (p_prev.head<2>() - local.back().head<2>()).norm() > 1e-6)
                        local.push_back(p_prev);
                }
                break;
            }

            graph_steps += seg_steps;
            steps_since_last_sample += seg_steps;

            if (steps_since_last_sample >= sample_steps || i == global_path.size() - 1)
            {
                if (local.empty() || (p.head<2>() - local.back().head<2>()).norm() > 1e-6)
                    local.push_back(p);
                steps_since_last_sample = 0;
            }
        }

        if (generation_verbose_.load())
        {
            if (!local.empty())
            {
                ROS_INFO("makeLocalAstarPathInAabb: done local_sz=%zu graph_steps=%d end=(%.3f,%.3f)", local.size(), graph_steps, local.back().x(), local.back().y());
            }
            else
            {
                ROS_INFO("makeLocalAstarPathInAabb: done local_sz=0 graph_steps=%d", graph_steps);
            }
        }


        // No fallback here: caller treats too-short as failure and waits for better odom/goal.

        return local;
    }

    inline bool generateAstarCorridorFromOdomHeading(std::vector<Eigen::Matrix<double, 6, -1>> &corridorSeq)
    {
        corridorSeq.clear();
        if (!odomInitialized.load() || !mapInitialized.load())
        {
            ROS_WARN("Cannot build A* corridor: missing odom or map");
            return false;
        }
        if (!glbMapPtr || !glbMapPtr->ogmPtr)
            return false;

        // snapshot shared inputs
        nav_msgs::Odometry odom_local;
        Eigen::Vector3d bound_min_local, bound_max_local;
        Eigen::Vector3d goal_local;
        {
            std::lock(odom_mutex_, bound_mutex_, status_mutex_);
            std::lock_guard<std::mutex> lk1(odom_mutex_, std::adopt_lock);
            std::lock_guard<std::mutex> lk2(bound_mutex_, std::adopt_lock);
            std::lock_guard<std::mutex> lk3(status_mutex_, std::adopt_lock);
            odom_local = odom;
            bound_min_local = bound_min;
            bound_max_local = bound_max;
            goal_local = latest_goal_;
        }

        Eigen::Vector3d start(odom_local.pose.pose.position.x,
                              odom_local.pose.pose.position.y,
                              odom_local.pose.pose.position.z);
        // force z to goal layer to keep planning in a consistent slice
        start.z() = goal_local.z();

        Eigen::Vector3d goal = goal_local;

        // Global A* should be searched ONCE for a static map and fixed goal, then reused.
        // Recompute only when goal changes.
        auto global_ptr = std::atomic_load(&latest_astar_global_path_ptr_);
        bool need_recompute = true;
        {
            std::lock_guard<std::mutex> lk(astar_cache_mutex_);
            need_recompute = !global_ptr || global_ptr->size() < 2 || !goalMatchesCachedAstarGoal(goal);
        }
        if (need_recompute)
        {
            std::vector<Eigen::Vector3d> global_path_new;
            if (!computeGlobalAstarPath(start, goal, bound_min_local, bound_max_local, global_path_new))
            {
                ROS_WARN("Global A* failed or empty path");
                return false;
            }
            auto sp = std::make_shared<std::vector<Eigen::Vector3d>>(std::move(global_path_new));
            std::atomic_store(&latest_astar_global_path_ptr_, sp);
            {
                std::lock_guard<std::mutex> lk(astar_cache_mutex_);
                astar_cached_goal_ = goal;
                astar_cached_goal_valid_ = true;
            }

            // New global path => reset progress so local cropping doesn't inherit stale indices.
            resetAstarProgress();
            global_ptr = sp;
        }

        if (!global_ptr || global_ptr->size() < 2)
            return false;

        std::vector<Eigen::Vector3d> local_path = makeLocalAstarPathInAabb(*global_ptr, start, odom_local, bound_min_local, bound_max_local);
        if (local_path.size() < 2)
        {
            ROS_WARN("Local A* path too short after AABB crop");
            return false;
        }
        {
            auto sp = std::make_shared<std::vector<Eigen::Vector3d>>(local_path);
            std::atomic_store(&latest_astar_local_path_ptr_, sp);
        }

        std::vector<Eigen::Matrix<double,6,-1>> subCorr;
        if (!corridorSeqGen(local_path, std::vector<Eigen::Matrix<double,6,-1>>(), subCorr))
        {
            ROS_WARN("corridorSeqGen failed for local A* path");
            return false;
        }

        corridorSeq.insert(corridorSeq.end(), subCorr.begin(), subCorr.end());
        return !corridorSeq.empty();
    }

    inline void initializeMapImpl(void)
    {
        pcl::PointCloud<pcl::PointXYZ> cloudDense;
        if (config.useLoadPCDFile)
        {
            if (!mapInitialized.load())
            {
                bound_min(0) = config.r3Bound[0];
                bound_min(1) = config.r3Bound[2];
                bound_min(2) = config.r3Bound[4];
                bound_max(0) = config.r3Bound[1];
                bound_max(1) = config.r3Bound[3];
                bound_max(2) = config.r3Bound[5];
                ROS_INFO("Initializing map from load file!");
                std::string path = ros::package::getPath("intention_get_corridor");
                pcl::io::loadPCDFile<pcl::PointXYZ>(path + config.pointCloudPath, cloudDense);

                glbMapPtr->initialize(cloudDense, config.outlierThreshold);
                pubMap(cloudDense);

                mapInitialized.store(true);
                glbMapPtr->getPointCloud(cloudDense, config.expectedHeight[1]);
            }
        }
    }

    inline void BoundCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (!config.useLoadPCDFile)
        {
            // 正常接收从MuJoCo Bridge发来的边界（现在基于R3Bound配置）
            std::lock_guard<std::mutex> lk(bound_mutex_);
            bound_min(0) = msg->pose.pose.position.x;
            bound_min(1) = msg->pose.pose.position.y;
            bound_min(2) = msg->pose.pose.position.z;

            bound_max(0) = msg->twist.twist.linear.x;
            bound_max(1) = msg->twist.twist.linear.y;
            bound_max(2) = msg->twist.twist.linear.z;

            std::cout << "Received bounds from MuJoCo Bridge: min = " << bound_min.transpose() << std::endl
                      << "                                    max = " << bound_max.transpose() << std::endl;
        }
    }

    inline void MapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (config.useLoadPCDFile)
            return;

        pcl::PointCloud<pcl::PointXYZ> cloudDense;
        ROS_INFO("Initializing map from callback!");
        pcl::fromROSMsg(*msg, cloudDense);
        glbMapPtr->initialize(cloudDense, config.outlierThreshold, bound_min, bound_max);
        pubMap(cloudDense);
        mapInitialized.store(true);

        glbMapPtr->getPointCloud(cloudDense, config.expectedHeight[1]);
        // preset auto-trigger removed; BFS/FOV corridor is used exclusively
    }

    inline void pubMap(const pcl::PointCloud<pcl::PointXYZ> &cloudDense, double sleep_time = 2)
    {
        sensor_msgs::PointCloud2 cloudVisMsg, cloudMsg;
        pcl::PointCloud<pcl::PointXYZ> infcloud;

        pcl::toROSMsg(cloudDense, cloudVisMsg);
        glbMapPtr->getPointCloud(infcloud, config.expectedHeight[1]);

        pcl::toROSMsg(infcloud, cloudMsg);

        cloudVisMsg.header.frame_id = "world";
        cloudMsg.header.frame_id = "world";
        ros::Rate sleep_rate(1 / sleep_time);
        sleep_rate.sleep();
        std::cout << "count = " << cloudDense.size() << " | " << infcloud.size() << std::endl;
        mapPub.publish(cloudMsg);
        visMapPub.publish(cloudVisMsg);

        ROS_WARN("Map has been published!");
    }

    inline void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        odom = *msg;

        const ros::Time stamp = (!msg->header.stamp.isZero()) ? msg->header.stamp : ros::Time::now();
        recent_odom_positions_.emplace_back(stamp,
                                            Eigen::Vector3d(msg->pose.pose.position.x,
                                                           msg->pose.pose.position.y,
                                                           msg->pose.pose.position.z));
        const ros::Time cutoff = stamp - ros::Duration(5.0);
        while (!recent_odom_positions_.empty() && recent_odom_positions_.front().first < cutoff) {
            recent_odom_positions_.pop_front();
        }
        odomInitialized.store(true);
    }

    // Callbacks for controller status
    inline void statusPredPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lk(status_mutex_);
        latest_pred_pose = *msg;
    }

    inline void statusPredVelCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lk(status_mutex_);
        latest_pred_vel = msg->vector;
    }

    inline void statusPredAccCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
    {
        // Currently not used, but stored for potential future use
        std::lock_guard<std::mutex> lk(status_mutex_);
        latest_pred_acc = msg->vector;
        // Placeholder: could store latest_pred_acc if needed
    }

    inline void extForceCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
    {
        if (use_time_based_collision_end_trigger_)
        {
            return;
        }

        double force_mag = Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z).norm();
        bool current_force_high = (force_mag > collision_external_force_threshold_);
        
        if (current_force_high)
        {
            is_force_high_.store(true);
            consecutive_low_force_count_ = 0;
        }
        else if (is_force_high_.load())
        {
            consecutive_low_force_count_++;
            if (consecutive_low_force_count_ >= collision_false_confirm_count_)
            {
                is_force_high_.store(false);
                consecutive_low_force_count_ = 0;
                
                // 仅当我们确实在执行碰撞轨迹时，才根据外力下降触发“碰撞完毕”的重规划
                if (isExecutingCollisionTrajectoryNow())
                {
                    handleSpecialSceneCollisionEndTrigger("special_scene_force_falling_edge",
                                                          special_scene_post_collision_wait_sec_);
                    return;
                }
            }
        }
    }

    inline void collisionTriggerCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (!use_time_based_collision_end_trigger_)
        {
            return;
        }

        const bool cur = msg->data;
        const bool prev = prev_collision_trigger_state_;

        collision_trigger_.store(cur);

        if (cur)
        {
            consecutive_collision_false_count_ = 0;
            prev_collision_trigger_state_ = true;
            // /collision_trigger now means "planned collision finished".
            // Trigger replanning on the rising edge as soon as the bridge reports
            // the end of the collision blend window.
            if (!prev)
            {
                if (handleSpecialSceneCollisionEndTrigger("special_scene_collision_trigger_rising_edge",
                                                          special_scene_post_collision_wait_sec_))
                {
                    return;
                }
            }
            return;
        }

        // Reset the latch once the short end-of-collision window disappears.
        prev_collision_trigger_state_ = false;
        consecutive_collision_false_count_ = 0;
    }

    inline void statusRemainingCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        std::unique_lock<std::mutex> lk(status_mutex_);
        latest_remaining_time = msg->data;
        lk.unlock();
        // Two-stage fixed-scene mode does not use remaining-time replanning.
    }

    inline void statusSegmentCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lk(status_mutex_);
        latest_segment_idx = msg->data;
    }

    inline bool incorridor(const Eigen::Matrix<double, 6, -1> &corridor, const Eigen::Vector3d &pt)
    {
        Eigen::Vector3d nor_vct, point;
        for (int i = 0; i < corridor.cols(); i++)
        {
            nor_vct = corridor.col(i).array().head(3);
            point = corridor.col(i).array().tail(3);

            double dot = nor_vct.dot(point - pt);
            if (dot < 0)
                return false;
        }
        return true;
    }

    inline bool checkInterCorridor(const Eigen::Matrix<double, 6, -1> cor1, const Eigen::Matrix<double, 6, -1> cor2)
    {
        Eigen::Matrix3Xd curIV;
        Eigen::Matrix<double, 6, -1> inter_corridor;
        inter_corridor.resize(6, cor1.cols() + cor2.cols());
        inter_corridor.leftCols(cor1.cols()) = cor1;
        inter_corridor.rightCols(cor2.cols()) = cor2;
        geoutils::enumerateVs(inter_corridor, curIV);
        return curIV.cols();
    }

    inline void CorridorShortCut(std::vector<Eigen::Matrix<double, 6, -1>> &corridorSeq)
    {
        // NOTE: Shortcut removal temporarily disabled for debugging.
        // It used to remove the middle polytope when non-adjacent polytopes intersect,
        // but that can accidentally remove legitimate intermediate SFCs in narrow passages.
        // To re-enable, restore the original logic above.
        if (corridorSeq.size() > 2)
        {
            ROS_DEBUG("CorridorShortCut: disabled (preserving all %zu polytopes)", corridorSeq.size());
        }
    }

    inline bool corridorSeqGen(const std::vector<Eigen::Vector3d> &pathList,
                               const std::vector<Eigen::Matrix<double, 6, -1>> &nowcorridor,
                               std::vector<Eigen::Matrix<double, 6, -1>> &corridorSeq)
    {
        corridorSeq.clear();
        Eigen::Matrix<double, 6, -1> cur_corridor;
        bool ret;
        if (nowcorridor.size())
        {
            cur_corridor = *(nowcorridor.end() - 1);
        }
        else
        {
            if (generation_verbose_.load()) ROS_INFO("corridorSeqGen: initial corridor_generate at start point (%.3f, %.3f, %.3f)",
                     pathList.front().x(), pathList.front().y(), pathList.front().z());
            ret = corridor_generate(*pathList.begin(), corridorSeq);
            if (!ret)
            {
                ROS_ERROR("Failed to generate corridor from start point.");
                return false;
            }
            if (generation_verbose_.load()) ROS_INFO("corridorSeqGen: initial polytope count=%zu", corridorSeq.size());
            cur_corridor = *corridorSeq.begin();
        }

        for (auto iter = pathList.begin(); iter != pathList.end(); iter++)
        {
            if (!incorridor(cur_corridor, *iter))
            {
                Eigen::Vector3d origin = *iter;
                if (generation_verbose_.load()) ROS_INFO("corridorSeqGen: need new polytope, origin=(%.3f, %.3f, %.3f)", origin.x(), origin.y(), origin.z());
                ret = corridor_generate(origin, corridorSeq);
                if (!ret)
                {
                    ROS_ERROR("Failed to generate corridor from start point.");
                    return false;
                }
                if (generation_verbose_.load()) ROS_INFO("corridorSeqGen: generated polytope, total polytopes now=%zu", corridorSeq.size());
                cur_corridor = *(corridorSeq.end() - 1);
            }
        }
        CorridorShortCut(corridorSeq);
        return true;
    }

    inline std::vector<Eigen::Matrix<double, 6, -1>> selectRequiredCorridors(
        const Eigen::Vector3d& start_point,
        const Eigen::Vector3d& end_point,
        const std::vector<Eigen::Matrix<double, 6, -1>>& full_corridor)
    {
        if (full_corridor.empty()) {
            ROS_WARN("Empty corridor provided for segment");
            return full_corridor;
        }

        // 辅助函数：检查点是否在走廊内
        auto isPointInCorridor = [](const Eigen::Matrix<double, 6, -1>& corridor_poly, const Eigen::Vector3d& pt) -> bool {
            for (int i = 0; i < corridor_poly.cols(); i++) {
                Eigen::Vector3d nor_vct = corridor_poly.col(i).head(3);
                Eigen::Vector3d point = corridor_poly.col(i).tail(3);
                double dot = nor_vct.dot(point - pt);
                if (dot < 0) {
                    return false;
                }
            }
            return true;
        };

        // 检查起点和终点在哪些走廊内
        std::vector<bool> start_in_corridor(full_corridor.size(), false);
        std::vector<bool> end_in_corridor(full_corridor.size(), false);
        
        for (size_t i = 0; i < full_corridor.size(); ++i) {
            start_in_corridor[i] = isPointInCorridor(full_corridor[i], start_point);
            end_in_corridor[i] = isPointInCorridor(full_corridor[i], end_point);
        }

        // 找到起点和终点所在的走廊范围
        int N = static_cast<int>(full_corridor.size());

        // 计算每个poly的质心（使用平面点的平均作为近似）
        std::vector<Eigen::Vector3d> centroids(N, Eigen::Vector3d::Zero());
        for (int i = 0; i < N; ++i) {
            const auto &poly = full_corridor[i];
            const int m = poly.cols();
            if (m <= 0) continue;
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            for (int j = 0; j < m; ++j) sum += poly.col(j).tail<3>();
            centroids[i] = sum / static_cast<double>(m);
        }

        // 方向向量（从起点指向终点）
        Eigen::Vector3d dir = end_point - start_point;
        double dir_norm = dir.norm();
        if (dir_norm < 1e-6) dir = Eigen::Vector3d(1, 0, 0);
        else dir /= dir_norm;

        // 选择最佳起点 corridor：在包含起点的集合中，取质心沿 dir 投影值最大的那个（最靠前）
        int chosen_start_idx = -1;
        double best_start_proj = -1e300;
        bool any_start = false;
        for (int i = 0; i < N; ++i) {
            if (!start_in_corridor[i]) continue;
            any_start = true;
            double proj = (centroids[i] - start_point).dot(dir);
            if (chosen_start_idx == -1 || proj > best_start_proj) {
                chosen_start_idx = i;
                best_start_proj = proj;
            }
        }

        // 选择最佳终点 corridor：在包含终点的集合中，取质心沿 dir 投影值最大的那个（最靠前）
        int chosen_end_idx = -1;
        double best_end_proj = -1e300;
        bool any_end = false;
        for (int i = 0; i < N; ++i) {
            if (!end_in_corridor[i]) continue;
            any_end = true;
            double proj = (centroids[i] - start_point).dot(dir);
            if (chosen_end_idx == -1 || proj > best_end_proj) {
                chosen_end_idx = i;
                best_end_proj = proj;
            }
        }

        int required_start = 0, required_end = N - 1;

        if (any_start && any_end) {
            required_start = std::min(chosen_start_idx, chosen_end_idx);
            required_end = std::max(chosen_start_idx, chosen_end_idx);
        } else if (any_start) {
            required_start = chosen_start_idx;
            // expand forward conservatively up to next one or one ahead
            required_end = std::min(chosen_start_idx + 1, N - 1);
        } else if (any_end) {
            required_end = chosen_end_idx;
            required_start = std::max(chosen_end_idx - 1, 0);
        } else {
            // none contain start/end: fallback to whole corridor
            required_start = 0;
            required_end = N - 1;
        }

        // 提取必要的走廊
        std::vector<Eigen::Matrix<double, 6, -1>> selected_corridors;
        for (int i = required_start; i <= required_end; ++i) {
            selected_corridors.push_back(full_corridor[i]);
        }

        // Debug info: 列出包含起点的索引和其投影值，便于定位起点多面体问题
        std::ostringstream oss;
        oss.setf(std::ios::fixed); oss<<std::setprecision(3);
        oss << "start_in_indices:[";
        for (int i = 0; i < N; ++i) {
            if (start_in_corridor[i]) {
                double proj = (centroids[i] - start_point).dot(dir);
                oss << i << "(p=" << proj << "),";
            }
        }
        oss << "] ";
        oss << "end_in_indices:[";
        for (int i = 0; i < N; ++i) {
            if (end_in_corridor[i]) {
                double proj = (centroids[i] - start_point).dot(dir);
                oss << i << "(p=" << proj << "),";
            }
        }
        oss << "] ";

        ROS_INFO("    Point (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f): selected corridors [%d:%d] (%lu total) %s",
                 start_point.x(), start_point.y(), start_point.z(),
                 end_point.x(), end_point.y(), end_point.z(),
                 required_start, required_end, selected_corridors.size(), oss.str().c_str());

        return selected_corridors;
    }

    inline bool generateFovCorridorFromOdomHeading(std::vector<Eigen::Matrix<double, 6, -1>> &corridorSeq)
    {
        ROS_WARN_THROTTLE(1.0, "BFS corridor generation is disabled (using A* pipeline)");
        (void)corridorSeq;
        return false;

#if 0
        if (!odomInitialized.load() || !mapInitialized.load())
        {
            ROS_WARN("Cannot build corridor: missing odom or map");
            return false;
        }

        const auto t0 = std::chrono::steady_clock::now();

        // snapshot shared inputs
        nav_msgs::Odometry odom_local;
        Eigen::Vector3d bound_min_local, bound_max_local;
        std::vector<std::pair<ros::Time, Eigen::Vector3d>> recent_odom_local;
        {
            std::lock(odom_mutex_, bound_mutex_);
            std::lock_guard<std::mutex> lk1(odom_mutex_, std::adopt_lock);
            std::lock_guard<std::mutex> lk2(bound_mutex_, std::adopt_lock);
            odom_local = odom;
            bound_min_local = bound_min;
            bound_max_local = bound_max;

            recent_odom_local.reserve(recent_odom_positions_.size());
            for (const auto &it : recent_odom_positions_) {
                recent_odom_local.push_back(it);
            }
        }

        Eigen::Vector3d base(odom_local.pose.pose.position.x,
                             odom_local.pose.pose.position.y,
                             odom_local.pose.pose.position.z);
        if (base.z() < config.expectedHeight[0]) base.z() = config.expectedHeight[0];
        else if (base.z() > config.expectedHeight[1]) base.z() = config.expectedHeight[1];
        // Search plane fixed at global goal z (user request)
        const double search_z = latest_goal_.z();

        auto ogm = glbMapPtr->ogmPtr;
        const double cellS = ogm->getScale();

        // convert base (projected to search_z) to grid index (fixed z-layer)
        Eigen::Vector3i baseIdx = ogm->convertPosD2I(Eigen::Vector3d(base.x(), base.y(), search_z));
        const int zIdx = baseIdx.z();

        // local BFS box: forward 8m, left/right 8m in vehicle body frame (will rotate)
        const double range_f = 8.0;
        const double range_lr = 8.0;

        // determine forward direction: prefer XY-velocity vector if available, otherwise use yaw
        double vx = 0.0, vy = 0.0;
        try { vx = odom_local.twist.twist.linear.x; vy = odom_local.twist.twist.linear.y; } catch(...) { vx = 0.0; vy = 0.0; }
        const double speed_thresh = 0.05; // m/s
        double yaw_dir = 0.0;
        if (std::hypot(vx, vy) > speed_thresh)
        {
            yaw_dir = std::atan2(vy, vx);
        }
        else
        {
            const auto &q = odom_local.pose.pose.orientation;
            yaw_dir = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        }

        // body axes in world frame
        const double cA = std::cos(yaw_dir);
        const double sA = std::sin(yaw_dir);
        Eigen::Vector2d ux(cA, sA);           // forward unit
        Eigen::Vector2d uy(-sA, cA);          // lateral unit (right-hand)

        // rotated rectangle corners (base + forward * {+range_f, -range_lr} + lateral * {+/-range_lr})
        Eigen::Vector2d base2(base.x(), base.y());
        std::array<Eigen::Vector2d,4> corners;
        corners[0] = base2 + ux * range_f + uy * range_lr;
        corners[1] = base2 + ux * range_f - uy * range_lr;
        corners[2] = base2 - ux * range_lr + uy * range_lr;
        corners[3] = base2 - ux * range_lr - uy * range_lr;

        double x_min = corners[0].x(), x_max = corners[0].x();
        double y_min = corners[0].y(), y_max = corners[0].y();
        for (int i = 1; i < 4; ++i)
        {
            x_min = std::min(x_min, corners[i].x());
            x_max = std::max(x_max, corners[i].x());
            y_min = std::min(y_min, corners[i].y());
            y_max = std::max(y_max, corners[i].y());
        }

        // clamp to world bounds
        x_min = std::max(bound_min_local.x(), x_min);
        x_max = std::min(bound_max_local.x(), x_max);
        y_min = std::max(bound_min_local.y(), y_min);
        y_max = std::min(bound_max_local.y(), y_max);

        Eigen::Vector3i idx_min = ogm->convertPosD2I(Eigen::Vector3d(x_min, y_min, search_z));
        Eigen::Vector3i idx_max = ogm->convertPosD2I(Eigen::Vector3d(x_max, y_max, search_z));

        int ix_min = std::min(idx_min.x(), idx_max.x());
        int ix_max = std::max(idx_min.x(), idx_max.x());
        int iy_min = std::min(idx_min.y(), idx_max.y());
        int iy_max = std::max(idx_min.y(), idx_max.y());

        // BFS over free cells within [ix_min,ix_max]x[iy_min,iy_max] at zIdx
        struct Cell { int x, y; };
        std::queue<Cell> q;
        std::unordered_map<long long, int> dist; // BFS graph distance (steps)
        std::unordered_map<long long, long long> parent; // parent pointer for backtracking

        auto keyOf = [](int x, int y)->long long { return ((long long)x << 32) | (unsigned long long)(y & 0xffffffff); };

        // inBox: first check AABB (idx bounds) then check rotated rectangle inclusion
        auto inBox = [&](int x, int y)->bool {
            if (x < ix_min || x > ix_max || y < iy_min || y > iy_max) return false;
            Eigen::Vector3d pos = ogm->convertPosI2D(Eigen::Vector3i(x, y, zIdx));
            Eigen::Vector2d dp(pos.x() - base.x(), pos.y() - base.y());
            double f = dp.x() * ux.x() + dp.y() * ux.y();
            double l = dp.x() * uy.x() + dp.y() * uy.y();
            return (f >= -range_lr && f <= range_f && l >= -range_lr && l <= range_lr);
        };

        int sx = baseIdx.x();
        int sy = baseIdx.y();
        if (!inBox(sx, sy))
        {
            ROS_WARN("Base index outside local BFS box");
            return false;
        }

        Eigen::Vector3i startIdx(sx, sy, zIdx);
        if (ogm->queryIdx(startIdx))
        {
            ROS_WARN("Base cell is occupied, searching for nearest free cell to start BFS...");
            bool found_free = false;
            int search_radius = std::ceil(1.0 / cellS); // search up to 1.0 meter
            for (int r = 1; r <= search_radius && !found_free; ++r) {
                for (int dx = -r; dx <= r && !found_free; ++dx) {
                    for (int dy = -r; dy <= r && !found_free; ++dy) {
                        if (std::abs(dx) == r || std::abs(dy) == r) {
                            if (inBox(sx + dx, sy + dy)) {
                                Eigen::Vector3i cand_idx(sx + dx, sy + dy, zIdx);
                                if (!ogm->queryIdx(cand_idx)) {
                                    sx = sx + dx;
                                    sy = sy + dy;
                                    startIdx = cand_idx;
                                    found_free = true;
                                    ROS_INFO("Found nearby free cell offset (%d, %d)", dx, dy);
                                }
                            }
                        }
                    }
                }
            }
            if (!found_free) {
                ROS_ERROR("Base cell is occupied and no free cell found within %.1fm, cannot start BFS", 1.0);
                return false;
            }
        }

        const int max_nodes = 500000;

        q.push({sx, sy});
        long long sKey = keyOf(sx, sy);
        dist[sKey] = 0;
        parent[sKey] = sKey;

        std::vector<Cell> visited_cells;
        visited_cells.reserve(1024);

        while (!q.empty() && (int)dist.size() < max_nodes)
        {
            Cell c = q.front();
            q.pop();
            visited_cells.push_back(c);

            const int cx = c.x, cy = c.y;
            const int cd = dist[keyOf(cx, cy)];

            const int nx4[4] = {1, -1, 0, 0};
            const int ny4[4] = {0, 0, 1, -1};
            for (int k = 0; k < 4; ++k)
            {
                int nx = cx + nx4[k];
                int ny = cy + ny4[k];
                if (!inBox(nx, ny)) continue;
                long long nk = keyOf(nx, ny);
                if (dist.find(nk) != dist.end()) continue;
                Eigen::Vector3i nIdx(nx, ny, zIdx);
                if (ogm->queryIdx(nIdx)) continue; // occupied or out-of-bounds
                dist[nk] = cd + 1;
                parent[nk] = keyOf(cx, cy);
                q.push({nx, ny});
            }
        }

        if (visited_cells.empty())
        {
            ROS_WARN("BFS visited no free cells");
            return false;
        }

        // Move dist into a shared_ptr so we can both reuse it locally and snapshot it for later plane selection.
        auto dist_sp = std::make_shared<std::unordered_map<long long, int>>();
        dist_sp->reserve(dist.size());
        dist_sp->swap(dist);
        const auto &dist_ref = *dist_sp;

        // (BFS visited visualization removed to save memory)

        // frontier detection: free cell with at least one neighbor outside visited or unknown/occupied
        struct FrontierCell { int x, y; int d; };
        std::vector<FrontierCell> frontiers;
        frontiers.reserve(256);

        auto isVisited = [&](int x, int y)->bool {
            return dist_ref.find(keyOf(x, y)) != dist_ref.end();
        };

        // Prepare goal grid check: if global latest_goal_ is inside visited free cells,
        // consider it a frontier (prefer direct goal reachability).
        // Use goal XY only (ignore goal z); project goal to current BFS z-layer for occupancy test
        Eigen::Vector3i goalIdxXY = ogm->convertPosD2I(Eigen::Vector3d(latest_goal_.x(), latest_goal_.y(), search_z));
        int goalX = goalIdxXY.x();
        int goalY = goalIdxXY.y();
        const bool goal_in_box = inBox(goalX, goalY);
        const bool goal_free = goal_in_box ? !ogm->queryIdx(Eigen::Vector3i(goalX, goalY, zIdx)) : false;
        const long long goalKey = keyOf(goalX, goalY);

        for (const auto &c : visited_cells)
        {
            int cx = c.x, cy = c.y;
            bool is_frontier = false;
            const int nx4[4] = {1, -1, 0, 0};
            const int ny4[4] = {0, 0, 1, -1};
            if (goal_free && isVisited(goalX, goalY) && keyOf(cx, cy) == goalKey) {
                is_frontier = true;
            }
            for (int k = 0; k < 4; ++k)
            {
                int nx = cx + nx4[k];
                int ny = cy + ny4[k];
                if (!inBox(nx, ny)) { is_frontier = true; break; }
                Eigen::Vector3i nIdx(nx, ny, zIdx);
                if (ogm->queryIdx(nIdx)) { continue; }
                if (!isVisited(nx, ny)) { continue; }
            }
            // If the visited cell equals the global goal grid and the goal is free and visited,
            // treat the goal as a frontier so the planner can go directly to it.
            if (is_frontier)
            {
                int d = dist_ref.at(keyOf(cx, cy));
                frontiers.push_back({cx, cy, d});
            }
        }

        if (frontiers.empty())
        {
            ROS_WARN("No frontier cells found in local BFS region");
            return false;
        }

        // (BFS frontier visualization removed to save memory)

        // cluster frontiers by 4-connectivity
        std::vector<int> labels(frontiers.size(), -1);
        int cluster_id = 0;

        for (size_t i = 0; i < frontiers.size(); ++i)
        {
            if (labels[i] != -1) continue;
            int cur_label = cluster_id++;
            std::queue<size_t> cq;
            cq.push(i);
            labels[i] = cur_label;

            while (!cq.empty())
            {
                size_t idx = cq.front(); cq.pop();
                int cx = frontiers[idx].x;
                int cy = frontiers[idx].y;
                for (size_t j = 0; j < frontiers.size(); ++j)
                {
                    if (labels[j] != -1) continue;
                    int dx = std::abs(frontiers[j].x - cx);
                    int dy = std::abs(frontiers[j].y - cy);
                    if (dx + dy == 1)
                    {
                        labels[j] = cur_label;
                        cq.push(j);
                    }
                }
            }
        }

        struct ClusterInfo
        {
            int id;
            int size;
            double avg_graph_dist;
            double max_graph_dist;
            double p90_graph_dist;
            int info_gain;
            Eigen::Vector3d centroid;
        };

        std::vector<ClusterInfo> clusters(cluster_id);
        std::vector<std::vector<int>> cluster_dists(cluster_id);
        for (int c = 0; c < cluster_id; ++c)
        {
            clusters[c].id = c;
            clusters[c].size = 0;
            clusters[c].avg_graph_dist = 0.0;
            clusters[c].max_graph_dist = 0.0;
            clusters[c].p90_graph_dist = 0.0;
            clusters[c].info_gain = 0;
            clusters[c].centroid.setZero();
        }

        for (size_t i = 0; i < frontiers.size(); ++i)
        {
            int lid = labels[i];
            if (lid < 0) continue;
            clusters[lid].size++;
            clusters[lid].avg_graph_dist += frontiers[i].d;
            clusters[lid].max_graph_dist = std::max(clusters[lid].max_graph_dist, (double)frontiers[i].d);
            cluster_dists[lid].push_back(frontiers[i].d);

            // simple information gain: count adjacent free-but-unvisited cells (or outside local box)
            const int nx4[4] = {1, -1, 0, 0};
            const int ny4[4] = {0, 0, 1, -1};
            for (int k = 0; k < 4; ++k)
            {
                int nx = frontiers[i].x + nx4[k];
                int ny = frontiers[i].y + ny4[k];
                if (!inBox(nx, ny)) { clusters[lid].info_gain += 1; continue; }
                Eigen::Vector3i nIdx(nx, ny, zIdx);
                if (ogm->queryIdx(nIdx)) { continue; }
                if (!isVisited(nx, ny)) { clusters[lid].info_gain += 1; }
            }

            Eigen::Vector3i idx(frontiers[i].x, frontiers[i].y, zIdx);
            Eigen::Vector3d wp = ogm->convertPosI2D(idx);
            wp.z() = base.z();
            clusters[lid].centroid += wp;
        }

        for (auto &cl : clusters)
        {
            if (cl.size > 0)
            {
                cl.avg_graph_dist /= (double)cl.size;
                cl.centroid /= (double)cl.size;

                auto &ds = cluster_dists[cl.id];
                if (!ds.empty())
                {
                    std::sort(ds.begin(), ds.end());
                    const size_t idx90 = (ds.size() <= 1) ? 0 : (size_t)std::floor(0.90 * (double)(ds.size() - 1));
                    cl.p90_graph_dist = (double)ds[idx90];
                }
            }
        }

        // score clusters: robust distance + info gain - revisit penalty
        struct ScoredCluster
        {
            double score;
            ClusterInfo info;
        };
        std::vector<ScoredCluster> scored;
        scored.reserve(clusters.size());

        const ros::Time now_stamp = (!odom_local.header.stamp.isZero()) ? odom_local.header.stamp : ros::Time::now();
        const ros::Time recent_cutoff = now_stamp - ros::Duration(5.0);
        const double revisit_radius_m = 3.0;
        const double w_dist = 1.0;
        const double w_size = 1.5;
        const double w_info = 1.0;
        const double w_revisit = 30.0;

        for (const auto &cl : clusters)
        {
            if (cl.size == 0) continue;
            const double graph_dist_m = cl.p90_graph_dist * cellS;

            double min_recent_dist = 1e9;
            for (const auto &rp : recent_odom_local)
            {
                if (rp.first < recent_cutoff) continue;
                const double dx = cl.centroid.x() - rp.second.x();
                const double dy = cl.centroid.y() - rp.second.y();
                const double d = std::hypot(dx, dy);
                if (d < min_recent_dist) min_recent_dist = d;
            }
            double revisit_penalty = 0.0;
            if (min_recent_dist < revisit_radius_m)
            {
                revisit_penalty = w_revisit * (1.0 - (min_recent_dist / revisit_radius_m));
            }

            const double score = w_dist * graph_dist_m
                                 + w_size * std::log((double)cl.size + 1.0)
                                 + w_info * std::log((double)cl.info_gain + 1.0)
                                 - revisit_penalty;
            scored.push_back({score, cl});
        }

        if (scored.empty())
        {
            ROS_WARN("All frontier clusters empty after scoring");
            return false;
        }

        std::sort(scored.begin(), scored.end(), [](const ScoredCluster &a, const ScoredCluster &b)
        {
            return a.score > b.score;
        });

        // Use the best cluster to extract a backtracked grid polyline from BFS parents
        const ClusterInfo &best_cl = scored.front().info;
        const int best_id = best_cl.id;

        // pick a representative frontier cell near p90 distance (avoid maxdist outliers)
        int rep_x = 0, rep_y = 0;
        int rep_target_d = (int)std::lround(best_cl.p90_graph_dist);
        int best_abs = std::numeric_limits<int>::max();
        int best_d = -1;
        for (size_t i = 0; i < frontiers.size(); ++i)
        {
            if (labels[i] != best_id) continue;
            const int d = frontiers[i].d;
            const int ad = std::abs(d - rep_target_d);
            if (ad < best_abs || (ad == best_abs && d > best_d))
            {
                best_abs = ad;
                best_d = d;
                rep_x = frontiers[i].x;
                rep_y = frontiers[i].y;
            }
        }

        // backtrack from rep cell to start using parent map
        long long repKey = keyOf(rep_x, rep_y);
        std::vector<std::pair<int,int>> rev;
        bool back_ok = true;
        long long cur = repKey;
        while (cur != sKey)
        {
            auto it = parent.find(cur);
            if (it == parent.end()) { back_ok = false; break; }
            int cx = (int)(cur >> 32);
            int cy = (int)(cur & 0xffffffff);
            rev.emplace_back(cx, cy);
            cur = it->second;
        }
        // always include start
        rev.emplace_back(sx, sy);
        std::reverse(rev.begin(), rev.end());

        // Map backtracked grid cells to world points using clearance-weighted averaging
        // This biases points toward regions with larger clearance (centerline of channel).
        std::vector<Eigen::Vector3d> path_world;
        if (back_ok && rev.size() >= 2)
        {
            const int neighbor_radius = 2; // tuneable: 1..3
            const int max_clear_cells = 4; // how far (in cells) to search for nearest obstacle
            const double eps_weight = 1e-3;
            path_world.reserve(rev.size());

            for (size_t i = 0; i < rev.size(); ++i)
            {
                int gx = rev[i].first;
                int gy = rev[i].second;

                Eigen::Vector3d weighted_sum = Eigen::Vector3d::Zero();
                double weight_sum = 0.0;

                for (int dx = -neighbor_radius; dx <= neighbor_radius; ++dx)
                {
                    for (int dy = -neighbor_radius; dy <= neighbor_radius; ++dy)
                    {
                        int nx = gx + dx;
                        int ny = gy + dy;
                        if (!inBox(nx, ny)) continue;
                        if (!isVisited(nx, ny)) continue; // only consider BFS-visited (free) cells

                        Eigen::Vector3i nbIdx(nx, ny, zIdx);
                        Eigen::Vector3d nbWorld = ogm->convertPosI2D(nbIdx);

                        // estimate clearance (approx): distance in cells to nearest occupied cell
                        double clearance = 0.0;
                        bool foundOcc = false;
                        for (int r = 0; r <= max_clear_cells && !foundOcc; ++r)
                        {
                            for (int sxr = -r; sxr <= r && !foundOcc; ++sxr)
                            {
                                for (int syr = -r; syr <= r && !foundOcc; ++syr)
                                {
                                    int cx = nx + sxr;
                                    int cy = ny + syr;
                                    if (!inBox(cx, cy)) continue;
                                    Eigen::Vector3i cidx(cx, cy, zIdx);
                                    if (ogm->queryIdx(cidx))
                                    {
                                        foundOcc = true;
                                    }
                                }
                            }
                            if (foundOcc)
                            {
                                clearance = r * cellS;
                                break;
                            }
                        }
                        if (!foundOcc)
                        {
                            clearance = (max_clear_cells + 1) * cellS;
                        }

                        double w = clearance + eps_weight;
                        weighted_sum += nbWorld * w;
                        weight_sum += w;
                    }
                }

                if (weight_sum > 0.0)
                {
                    Eigen::Vector3d pw = weighted_sum / weight_sum;
                    pw.z() = base.z();
                    path_world.push_back(pw);
                }
                else
                {
                    Eigen::Vector3d pw = ogm->convertPosI2D(Eigen::Vector3i(gx, gy, zIdx));
                    pw.z() = base.z();
                    path_world.push_back(pw);
                }
            }
        }

        // fallback: if backtracking failed or path too short, use centroid as simple two-point path
        if (path_world.size() < 2)
        {
            ROS_WARN("Backtracking path too short or failed, falling back to centroid seed");
            Eigen::Vector3d c = best_cl.centroid;
            bool free_ok = false;
            try { free_ok = glbMapPtr->safeQuery(c); } catch(...) { free_ok = false; }
            if (!free_ok)
            {
                ROS_WARN("Centroid also invalid, aborting BFS corridor generation");
                return false;
            }
            path_world.clear();
            path_world.push_back(base);
            path_world.push_back(c);
        }

        // simplify path (remove near-collinear points)
        std::vector<Eigen::Vector3d> simplified;
        for (size_t i = 0; i < path_world.size(); ++i)
        {
            if (i == 0 || i + 1 == path_world.size()) simplified.push_back(path_world[i]);
            else
            {
                Eigen::Vector3d a = path_world[i-1], b = path_world[i], c = path_world[i+1];
                Eigen::Vector3d ab = (b - a).normalized();
                Eigen::Vector3d bc = (c - b).normalized();
                if ((ab - bc).norm() > 1e-3) simplified.push_back(b);
            }
        }

        // --- adjust points toward geometric centerline between obstacles using lateral rays ---
        auto adjustToCenterline = [&](std::vector<Eigen::Vector3d> &pts)
        {
            if (pts.size() < 3) return;
            const double ray_step = cellS;        // step size along normal
            const int max_ray_steps = 8;          // maximum steps to search each side

            for (size_t i = 1; i + 1 < pts.size(); ++i)
            {
                Eigen::Vector3d p = pts[i];
                // compute tangent using neighbors
                Eigen::Vector3d t = (pts[i+1] - pts[i-1]);
                t.z() = 0.0;
                if (t.head<2>().norm() < 1e-4) continue;
                t.normalize();

                // 2D normal (rotate tangent by +90 deg in xy)
                Eigen::Vector3d n;
                n.x() = -t.y();
                n.y() =  t.x();
                n.z() = 0.0;

                auto castRay = [&](const Eigen::Vector3d &origin, const Eigen::Vector3d &dir, Eigen::Vector3d &hit)->bool
                {
                    Eigen::Vector3d cur = origin;
                    for (int s = 1; s <= max_ray_steps; ++s)
                    {
                        cur += dir * ray_step;
                        Eigen::Vector3i idx = ogm->convertPosD2I(cur);
                        if (!inBox(idx.x(), idx.y())) return false; // out of local BFS box
                        if (ogm->queryIdx(idx))
                        {
                            hit = cur;
                            return true;
                        }
                    }
                    return false;
                };

                Eigen::Vector3d hitL, hitR;
                bool okL = castRay(p,  n, hitL);
                bool okR = castRay(p, -n, hitR);

                if (okL && okR)
                {
                    Eigen::Vector3d mid = 0.5 * (hitL + hitR);
                    pts[i].x() = mid.x();
                    pts[i].y() = mid.y();
                }
            }
        };

        adjustToCenterline(simplified);

        // sample along simplified (and centerline-adjusted) polyline to produce multiple path points
        const double sample_step = 0.8; // meters
        std::vector<Eigen::Vector3d> pathList;
        pathList.push_back(simplified.front());
        const double max_total_len = 13.0; // meters cap requested by user
        double acc_len = 0.0;
        pathList.push_back(simplified.front());
        bool reached_cap = false;

        for (size_t seg = 0; seg + 1 < simplified.size() && !reached_cap; ++seg)
        {
            Eigen::Vector3d p0 = simplified[seg], p1 = simplified[seg+1];
            double seglen = (p1 - p0).norm();
            if (seglen < 1e-6) continue;
            int n = std::max(1, (int)std::floor(seglen / sample_step));
            for (int k = 1; k <= n; ++k)
            {
                double t = (double)k / (double)(n + 0);
                Eigen::Vector3d pt = p0 + (p1 - p0) * t;

                // incremental length from last added point
                double add_len = (pt - pathList.back()).norm();

                if (acc_len + add_len <= max_total_len + 1e-9)
                {
                    pathList.push_back(pt);
                    acc_len += add_len;
                }
                else
                {
                    // 超出 15m，按你的要求不插值，直接退出（不加入此点）
                    reached_cap = true;
                    break;
                }
            }
        }
        if (generation_verbose_.load()) {
            ROS_INFO("BFS backtracked pathList size=%zu (arc_len=%.3f m)", pathList.size(), acc_len);
            for (size_t ii = 0; ii < pathList.size(); ++ii)
            {
                ROS_INFO("  pathList[%zu]= %.3f, %.3f, %.3f", ii, pathList[ii].x(), pathList[ii].y(), pathList[ii].z());
            }
        }

        // generate corridors from the multi-point pathList
        corridorSeq.clear();
        std::vector<Eigen::Matrix<double,6,-1>> subCorr;

        // Cache sampled BFS path points for later direction fallback.
        {
            auto path_sp = std::make_shared<std::vector<Eigen::Vector3d>>(pathList);
            std::atomic_store(&latest_bfs_path_ptr_, path_sp);
        }

        if (!corridorSeqGen(pathList, std::vector<Eigen::Matrix<double,6,-1>>(), subCorr))
        {
            ROS_WARN("corridorSeqGen failed for backtracked path");
            return false;
        }
        corridorSeq.insert(corridorSeq.end(), subCorr.begin(), subCorr.end());

        // Update BFS dist snapshot paired with this corridor generation.
        {
            auto bfs = std::make_shared<BfsDistSnapshot>();
            bfs->ix_min = ix_min;
            bfs->ix_max = ix_max;
            bfs->iy_min = iy_min;
            bfs->iy_max = iy_max;
            bfs->zIdx = zIdx;
            bfs->cellS = cellS;
            bfs->base_z = base.z();
            bfs->dist = dist_sp;
            std::atomic_store(&latest_bfs_ptr_, bfs);
        }

        const auto t1 = std::chrono::steady_clock::now();
        const double elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();
        if (generation_verbose_.load()) ROS_INFO("BFS-frontier corridor built in %.3f ms (seeds=%lu, polytopes=%lu)",
                                                elapsed_ms, pathList.size(), corridorSeq.size());

        return !corridorSeq.empty();
#endif
    }

    inline void loadPosAtt(Eigen::Matrix3Xd &keypos, quadrotor_msgs::TrajectoryPlan &plan_msg, const Eigen::Vector3d target)
    {
        plan_msg.header.frame_id = "world";
        plan_msg.header.stamp = ros::Time::now();
        plan_msg.trajectory_mode = quadrotor_msgs::TrajectoryPlan::MODE_UNSET;
        plan_msg.segment_times.clear();

        keypos.resize(3, config.cnt_pos + 1);
        keypos.setZero();

        plan_msg.waypoints.resize(config.cnt_pos + 1);
        plan_msg.headings.resize(config.cnt_pos + 1);
        plan_msg.position_constraints.resize(config.cnt_pos + 1);

        // Snapshot odom to avoid races
        nav_msgs::Odometry odom_local;
        {
            std::lock_guard<std::mutex> lk(odom_mutex_);
            odom_local = odom;
        }

        keypos(0, 0) = odom_local.pose.pose.position.x;
        keypos(1, 0) = odom_local.pose.pose.position.y;
        keypos(2, 0) = odom_local.pose.pose.position.z;

        plan_msg.waypoints[0].x = keypos(0, 0);
        plan_msg.waypoints[0].y = keypos(1, 0);
        plan_msg.waypoints[0].z = keypos(2, 0);
        plan_msg.headings[0].x = 0.0;
        plan_msg.headings[0].y = 0.0;
        plan_msg.headings[0].z = 1.0;
        plan_msg.position_constraints[0] = false;

        auto it_pos = config.set_pos.begin();
        auto it_att = config.set_att.begin();
        Eigen::Vector3d waypoint_heading_unit;
        double position_constraint_flag;
        for (int i = 1; i < config.cnt_pos + 1; i++)
        {
            keypos(0, i) = *it_pos++;
            keypos(1, i) = *it_pos++;
            keypos(2, i) = *it_pos++;

            waypoint_heading_unit(0) = *it_att++;
            waypoint_heading_unit(1) = *it_att++;
            waypoint_heading_unit(2) = *it_att++;
            waypoint_heading_unit.normalize();
            position_constraint_flag = *it_att++;

            plan_msg.waypoints[i].x = keypos(0, i);
            plan_msg.waypoints[i].y = keypos(1, i);
            plan_msg.waypoints[i].z = keypos(2, i);
            plan_msg.headings[i].x = waypoint_heading_unit(0);
            plan_msg.headings[i].y = waypoint_heading_unit(1);
            plan_msg.headings[i].z = waypoint_heading_unit(2);
            plan_msg.position_constraints[i] = (position_constraint_flag != 0.0);
        }
    }

    inline void setposCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        static int seq = 0;
        if (!odomInitialized.load())
        {
            ROS_WARN("No Odom!");
            return;
        }
        quadrotor_msgs::TrajectoryPlan plan_msg;
        Eigen::Matrix3Xd key_pos;
        Eigen::Vector3d target;
        target(0) = msg->pose.position.x;
        target(1) = msg->pose.position.y;
        target(2) = msg->pose.position.z;
        loadPosAtt(key_pos, plan_msg, target);

        {
            // 记录最新 goal，并在收到新 goal 时允许重新规划
            std::lock_guard<std::mutex> lk(status_mutex_);
            latest_goal_ = target;
        }

        // New goal resets collision-candidate latch.
        collision_candidate_active_.store(false);
        // New goal cancels collision execution state.
        executing_collision_traj_.store(false);
        collision_exec_until_sec_.store(0.0);
        suppress_time_replan_until_sec_.store(0.0);
        resetSpecialSceneStateForNewGoal();

        plan_msg.header.seq = ++seq;
        // Only publish to /MyPointSeq when explicitly configured to load PCD (preserve original behavior).
        if (config.useLoadPCDFile)
        {
            intentionPub.publish(plan_msg);
        }
        getCorridorFromSetPos(plan_msg, key_pos);
    }

    void getCorridorFromSetPos(quadrotor_msgs::TrajectoryPlan &plan_msg, const Eigen::Matrix3Xd & /*key_pos*/)
    {
        (void)plan_msg;

        if (!ensureGlobalAstarPathAndFullCorridorBuilt())
        {
            ROS_WARN("getCorridorFromSetPos: global path/corridor not ready");
            return;
        }

        // Snapshot current odom/bounds + controller predicted status (for generating avoidance trajectory)
        nav_msgs::Odometry odom_local;
        Eigen::Vector3d bound_min_local, bound_max_local;
        geometry_msgs::PoseStamped pred_pose;
        geometry_msgs::Vector3 pred_vel;
        geometry_msgs::Vector3 pred_acc;
        Eigen::Vector3d goal_local;
        Eigen::Vector3d current_traj_goal_local = Eigen::Vector3d::Zero();
        double remaining_time_local = 0.0;
        {
            std::lock(odom_mutex_, bound_mutex_, status_mutex_);
            std::lock_guard<std::mutex> lk1(odom_mutex_, std::adopt_lock);
            std::lock_guard<std::mutex> lk2(bound_mutex_, std::adopt_lock);
            std::lock_guard<std::mutex> lk3(status_mutex_, std::adopt_lock);
            odom_local = odom;
            bound_min_local = bound_min;
            bound_max_local = bound_max;
            pred_pose = latest_pred_pose;
            pred_vel = latest_pred_vel;
            pred_acc = latest_pred_acc;
            goal_local = latest_goal_;
            remaining_time_local = latest_remaining_time;
            current_traj_goal_local = current_traj_goal_;
        }

        auto global_ptr = std::atomic_load(&latest_astar_global_path_ptr_);
        auto full_corridor_ptr = std::atomic_load(&latest_corridor_ptr_);
        if (!global_ptr || global_ptr->size() < 2 || !full_corridor_ptr || full_corridor_ptr->empty())
        {
            ROS_WARN("getCorridorFromSetPos: cached global path or full corridor empty");
            return;
        }

        // Build LOCAL path by cropping the cached global path around current position.
        // Use predicted odom at trajectory start delay (ROS param 'TrajStartDelay') instead of current odom
        nav_msgs::Odometry odom_future = odom_local;
        try {
            double traj_delay = 0.05;
            odom_future.pose.pose.position.x = pred_pose.pose.position.x + pred_vel.x * traj_delay + 0.5 * pred_acc.x * traj_delay * traj_delay;
            odom_future.pose.pose.position.y = pred_pose.pose.position.y + pred_vel.y * traj_delay + 0.5 * pred_acc.y * traj_delay * traj_delay;
            odom_future.pose.pose.position.z = pred_pose.pose.position.z + pred_vel.z * traj_delay + 0.5 * pred_acc.z * traj_delay * traj_delay;
            odom_future.twist.twist.linear.x = pred_vel.x + pred_acc.x * traj_delay;
            odom_future.twist.twist.linear.y = pred_vel.y + pred_acc.y * traj_delay;
            odom_future.twist.twist.linear.z = pred_vel.z + pred_acc.z * traj_delay;
        } catch(...) {
            // fallback to current odom if prediction missing
            odom_future = odom_local;
        }

        Eigen::Vector3d base(odom_future.pose.pose.position.x,
                             odom_future.pose.pose.position.y,
                             odom_future.pose.pose.position.z);
        base.z() = goal_local.z();

        std::vector<Eigen::Vector3d> local_path = makeLocalAstarPathInAabb(*global_ptr, base, odom_future, bound_min_local, bound_max_local);
        if (local_path.size() < 2)
        {
            ROS_WARN("getCorridorFromSetPos: local path too short after AABB crop");
            return;
        }
        {
            auto sp = std::make_shared<std::vector<Eigen::Vector3d>>(local_path);
            std::atomic_store(&latest_astar_local_path_ptr_, sp);
        }

        // Select corridor subset that covers the local path points.
        std::vector<Eigen::Matrix<double, 6, -1>> sel = selectCorridorsCoveringPoints(local_path, *full_corridor_ptr);
        if (sel.empty())
        {
            sel = selectRequiredCorridors(base, local_path.back(), *full_corridor_ptr);
        }
        const auto &to_pub = (!sel.empty()) ? sel : (*full_corridor_ptr);

        const SpecialSceneReplanStage special_stage = getSpecialSceneStage();
        const bool do_special_initial = (special_stage == SpecialSceneReplanStage::AwaitInitialCollisionPlan);
        const bool do_special_final =
            (special_stage == SpecialSceneReplanStage::AwaitFinalAvoidance) || collision_event_trigger_.load();
        if (!do_special_initial && !do_special_final)
        {
            return;
        }

        const auto *active_corridors = &to_pub;
        std::vector<Eigen::Matrix<double, 6, -1>> final_stage_corridors;
        if (do_special_final)
        {
            final_stage_corridors = selectRequiredCorridors(base, goal_local, *full_corridor_ptr);
            if (!final_stage_corridors.empty())
            {
                active_corridors = &final_stage_corridors;
            }
        }

        auto plane_candidates = pubCorridor(*active_corridors);
        visualizer.visualizePolytope(*active_corridors);
        const bool has_collision = do_special_initial ? updateSelectedUnverifiedCollisionPlaneByKinematics(plane_candidates) : false;

        geometry_msgs::Vector3 att;
        att.x = 0.0; att.y = 0.0; att.z = 0.0;

        // Stage-1 follows the local path heading; stage-2 goes directly from current point to the global goal.
        Eigen::Vector3d preferred_target = goal_local;
        Eigen::Vector3d preferred_dir_unit(1.0, 0.0, 0.0);
        if (do_special_final)
        {
            Eigen::Vector3d d = goal_local - base;
            d.z() = 0.0;
            if (d.head<2>().norm() > 1e-6) preferred_dir_unit = d.normalized();
        }
        else if (local_path.size() >= 2)
        {
            Eigen::Vector3d d = local_path.back() - local_path[local_path.size() - 2];
            d.z() = 0.0;
            if (d.head<2>().norm() > 1e-6) preferred_dir_unit = d.normalized();
            preferred_target = local_path.back();
        }

        preferred_target.z() = goal_local.z();
        ROS_WARN("PREFERRED TARGET: (%.3f, %.3f, %.3f),", preferred_target.x(), preferred_target.y(), preferred_target.z());
        quadrotor_msgs::TrajectoryPlan forward_plan;
        const double plan_now_sec = ros::Time::now().toSec();

        if (special_stage == SpecialSceneReplanStage::AwaitInitialCollisionPlan)
        {
            if (!has_collision)
            {
                ROS_WARN("Special scene stage-1: no unverified collision plane selected, cannot publish collision trajectory");
                return;
            }

            const Eigen::Vector3d n = selected_collision_plane_normal_;
            const double DR = this->config.dilateRadius;
            const double GR = (glbMapPtr && glbMapPtr->ogmPtr) ? glbMapPtr->getScale() : this->config.gridResolution;
            const int r = static_cast<int>(std::ceil(DR / GR));
            const double D = r * GR;
            const double O = 0.5 * GR;
            const double adjust = (D + O - DR) + 0.01;
            const double d_obs = n.dot(selected_collision_plane_point_);
            const double d = d_obs + adjust;

            quadrotor_msgs::CollisionTrajectory ct;
            bool has_ct = false;
            forward_plan = sample_forward::generateMixedTrajectory(pred_pose, pred_vel, pred_acc, att,
                                  plane_candidates, goal_local,
                                  preferred_target, preferred_dir_unit,
                                  this->config, n, d, selected_collision_plane_width_,
                                  selected_collision_plane_point_, &ct, &has_ct);

            if (has_ct && forward_plan.trajectory_mode == quadrotor_msgs::TrajectoryPlan::MODE_COLLISION)
            {
                std::lock_guard<std::mutex> lk(this->last_collision_mutex_);
                this->last_collision_traj_ = ct;
                this->has_last_collision_traj_ = true;
            }

            const bool is_collision_mode =
                (has_ct && forward_plan.trajectory_mode == quadrotor_msgs::TrajectoryPlan::MODE_COLLISION);
            if (!is_collision_mode)
            {
                ROS_WARN("Special scene stage-1: selected collision plane exists, but generated plan is not MODE_COLLISION");
                return;
            }

            collision_candidate_active_.store(true);
            executing_collision_traj_.store(true);
            collision_exec_until_sec_.store(plan_now_sec + std::max(0.1, collision_exec_hold_sec_));
            suppress_time_replan_until_sec_.store(plan_now_sec + std::max(0.1, collision_exec_hold_sec_));
            special_scene_stage_.store(static_cast<int>(SpecialSceneReplanStage::AwaitCollisionEnd));
            ROS_WARN("Special scene stage-1: publish collision plan using selected plane n=(%.3f, %.3f, %.3f) d=%.3f width=%.3f point=(%.3f, %.3f, %.3f)",
                     n.x(), n.y(), n.z(), -d_obs, selected_collision_plane_width_,
                     selected_collision_plane_point_.x(), selected_collision_plane_point_.y(), selected_collision_plane_point_.z());
        }
        else
        {
            forward_plan = buildSpecialSceneFinalAstarPlan(*global_ptr, base, goal_local, preferred_dir_unit);
            collision_candidate_active_.store(false);
            executing_collision_traj_.store(false);
            collision_exec_until_sec_.store(0.0);
            special_scene_stage_.store(static_cast<int>(SpecialSceneReplanStage::Completed));
            ROS_WARN("Special scene stage-2: publishing final A* waypoint plan to global goal with %zu waypoints",
                     forward_plan.waypoints.size());
        }

        keyPosPub.publish(forward_plan);
        intentionPub.publish(forward_plan);
        {
            std::lock_guard<std::mutex> lk(this->status_mutex_);
            if (!forward_plan.waypoints.empty())
            {
                const auto &wp = forward_plan.waypoints.back();
                this->current_traj_goal_ = Eigen::Vector3d(wp.x, wp.y, wp.z);
            }
            else
            {
                this->current_traj_goal_ = preferred_target;
            }
        }
        last_collision_trigger_time_ = ros::Time::now();
        publishCompleteCollisionTrajectory();
        collision_event_trigger_.store(false);
    }

    inline bool selectVerifiedFrontierPlaneByBfsDistance(const std::vector<PlaneCandidate> &candidates,
                                                         const nav_msgs::Odometry &odom_local,
                                                         PlaneCandidate &best_cand,
                                                         int &best_steps)
    {
        best_steps = -1;
        (void)odom_local;
        if (!mapInitialized.load() || !glbMapPtr || !glbMapPtr->ogmPtr)
        {
            return false;
        }

        bool any_verified = false;
        for (const auto &c : candidates)
        {
            if (c.verified_frontier) { any_verified = true; break; }
        }
        if (!any_verified)
        {
            return false; // explicit: no fallback if none verified
        }

        // Reuse BFS dist snapshot from corridor generation; do NOT recompute BFS here.
        auto bfs_ptr = std::atomic_load(&latest_bfs_ptr_);
        if (!bfs_ptr || !bfs_ptr->dist)
        {
            return false;
        }

        auto ogm = glbMapPtr->ogmPtr;
        const double cellS = bfs_ptr->cellS > 0.0 ? bfs_ptr->cellS : ogm->getScale();
        const int zIdx = bfs_ptr->zIdx;

        // Fallback support: if a candidate has no direct BFS dist entry, estimate its total steps as
        //   steps(end) + ceil(dist(end, plane)/cellS).
        int end_steps = -1;
        Eigen::Vector3d bfs_end = Eigen::Vector3d::Zero();
        {
            auto path_ptr = std::atomic_load(&latest_bfs_path_ptr_);
            if (path_ptr && !path_ptr->empty())
            {
                bfs_end = path_ptr->back();
                bfs_end.z() = bfs_ptr->base_z;
                Eigen::Vector3i end_idx = ogm->convertPosD2I(bfs_end);
                end_idx.z() = zIdx;

                auto keyOfTmp = [](int x, int y)->long long { return ((long long)x << 32) | (unsigned long long)(y & 0xffffffff); };
                auto it = bfs_ptr->dist->find(keyOfTmp(end_idx.x(), end_idx.y()));
                if (it != bfs_ptr->dist->end())
                {
                    end_steps = it->second;
                }
            }
        }

        // If the path end does not map to a BFS dist entry (due to smoothing/averaging), use the
        // maximum dist seen in the snapshot as a conservative endpoint step count.
        if (end_steps < 0)
        {
            int max_steps = -1;
            for (const auto &kv : *(bfs_ptr->dist))
            {
                if (kv.second > max_steps) max_steps = kv.second;
            }
            end_steps = max_steps;
        }

        auto keyOf = [](int x, int y)->long long { return ((long long)x << 32) | (unsigned long long)(y & 0xffffffff); };
        auto inBox = [&](int x, int y)->bool {
            return x >= bfs_ptr->ix_min && x <= bfs_ptr->ix_max && y >= bfs_ptr->iy_min && y <= bfs_ptr->iy_max;
        };

        const double eps = std::max(0.05, cellS * 0.5);
        const double push = config.dilateRadius + eps;
        const double offsets[5] = {0.0, eps, -eps, push, -push};

        for (const auto &cand : candidates)
        {
            if (!cand.verified_frontier) continue;

            int cand_best = -1;
            for (int oi = 0; oi < 5; ++oi)
            {
                Eigen::Vector3d p = cand.point_on_plane + offsets[oi] * cand.normal_unit;
                p.z() = bfs_ptr->base_z;
                Eigen::Vector3i idx = ogm->convertPosD2I(p);
                idx.z() = zIdx;
                if (!inBox(idx.x(), idx.y())) continue;
                auto it = bfs_ptr->dist->find(keyOf(idx.x(), idx.y()));
                if (it == bfs_ptr->dist->end()) continue;
                if (it->second > cand_best) cand_best = it->second;
            }

            // Fallback: candidate not covered by BFS dist map (e.g., beyond BFS box), estimate steps
            // using BFS endpoint + Euclidean distance from endpoint to plane.
            if (cand_best < 0 && end_steps >= 0)
            {
                Eigen::Vector3d plane_pt = cand.point_on_plane;
                plane_pt.z() = bfs_ptr->base_z;
                Eigen::Vector3d end_pt = bfs_end;
                if (!end_pt.allFinite() || end_pt.isZero(1e-12))
                {
                    // fallback to current odom if no BFS end point available
                    end_pt = Eigen::Vector3d(odom_local.pose.pose.position.x,
                                             odom_local.pose.pose.position.y,
                                             bfs_ptr->base_z);
                }

                const double dist_plane_m = std::fabs(cand.normal_unit.dot(end_pt - plane_pt));
                const int add_steps = (cellS > 1e-9) ? (int)std::ceil(dist_plane_m / cellS) : 0;
                cand_best = end_steps + add_steps;
            }

            if (cand_best > best_steps)
            {
                best_steps = cand_best;
                best_cand = cand;
            }
        }

        return best_steps >= 0;
    }

    inline bool updateSelectedVerifiedFrontierPlaneByBfs(const std::vector<PlaneCandidate> &candidates)
    {
        // Snapshot odom to avoid races
        nav_msgs::Odometry odom_local;
        {
            std::lock_guard<std::mutex> lk(odom_mutex_);
            odom_local = odom;
        }

        PlaneCandidate best;
        int best_steps = -1;
        const bool ok = selectVerifiedFrontierPlaneByBfsDistance(candidates, odom_local, best, best_steps);

        {
            std::lock_guard<std::mutex> lk(selected_frontier_plane_mutex_);
            if (ok)
            {
                has_selected_frontier_plane_ = true;
                selected_frontier_plane_normal_ = best.normal_unit;
                selected_frontier_plane_point_ = best.point_on_plane;
                selected_frontier_plane_width_ = best.width;
                selected_frontier_plane_corridor_idx_ = best.corridor_idx;
                selected_frontier_plane_face_idx_ = best.face_idx;
                selected_frontier_plane_bfs_steps_ = best_steps;
            }
            else
            {
                has_selected_frontier_plane_ = false;
                selected_frontier_plane_normal_.setZero();
                selected_frontier_plane_point_.setZero();
                selected_frontier_plane_width_ = 0.0;
                selected_frontier_plane_corridor_idx_ = -1;
                selected_frontier_plane_face_idx_ = -1;
                selected_frontier_plane_bfs_steps_ = -1;
            }
        }

        if (ok)
        {
            // compute plane constant d and distance from current odom to plane
            Eigen::Vector3d odom_pos(odom_local.pose.pose.position.x,
                                     odom_local.pose.pose.position.y,
                                     odom_local.pose.pose.position.z);
            Eigen::Vector3d n = best.normal_unit;
            double d = -(n.dot(best.point_on_plane));
            double dist_m = std::fabs(n.dot(odom_pos) + d);
            ROS_INFO("Selected verified frontier plane: corridor=%d face=%d n=(%.3f, %.3f, %.3f) d=%.3f width=%.3f dist=%.3f",
                     best.corridor_idx, best.face_idx,
                     n.x(), n.y(), n.z(), d, best.width, dist_m);
        }
        else
        {
            ROS_INFO("No selectable verified frontier plane by BFS (return None)");
        }
        return ok;
    }

    inline bool updateSelectedVerifiedFrontierPlaneByAstar(const std::vector<PlaneCandidate> &candidates,
                                                           const Eigen::Vector3d &local_path_end)
    {
        // Mirror the original BFS selection logic (comprehensive, not just nearest):
        // - Only consider candidates with verified_frontier==true
        // - Pick the candidate that is the MOST forward (max "steps"), where steps are approximated
        //   by progress along the latest local A* path
        // - Use multiple offset samples along the plane normal (same as BFS) and a fallback estimate
        //   when no sample is close to the path

        // Snapshot odom for logging / distance computation
        nav_msgs::Odometry odom_local;
        {
            std::lock_guard<std::mutex> lk(odom_mutex_);
            odom_local = odom;
        }

        // Need a local A* path snapshot to approximate BFS steps.
        auto path_ptr = std::atomic_load(&latest_astar_local_path_ptr_);
        if (!path_ptr || path_ptr->size() < 2)
        {
            std::lock_guard<std::mutex> lk(selected_frontier_plane_mutex_);
            has_selected_frontier_plane_ = false;
            selected_frontier_plane_normal_.setZero();
            selected_frontier_plane_point_.setZero();
            selected_frontier_plane_width_ = 0.0;
            selected_frontier_plane_corridor_idx_ = -1;
            selected_frontier_plane_face_idx_ = -1;
            selected_frontier_plane_bfs_steps_ = -1;
            return false;
        }

        // Cell size for translating meters -> steps (keeps logs/field consistent with BFS naming).
        double cellS = this->config.gridResolution;
        if (glbMapPtr && glbMapPtr->ogmPtr)
        {
            const double s = glbMapPtr->ogmPtr->getScale();
            if (s > 1e-9) cellS = s;
        }
        if (cellS <= 1e-9) cellS = 0.2;

        // Precompute cumulative GRAPH steps along local A* path in XY (not arc length).
        // Use Chebyshev steps to match 8-neighborhood grid graph distance.
        auto segStepsChebyshev = [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b)->int {
            const double dx = std::fabs(b.x() - a.x());
            const double dy = std::fabs(b.y() - a.y());
            const double m = std::max(dx, dy);
            if (cellS <= 1e-9) return 1;
            return std::max(1, (int)std::ceil(m / cellS));
        };

        std::vector<int> steps_acc;
        steps_acc.resize(path_ptr->size(), 0);
        for (size_t i = 1; i < path_ptr->size(); ++i)
        {
            steps_acc[i] = steps_acc[i - 1] + segStepsChebyshev((*path_ptr)[i - 1], (*path_ptr)[i]);
        }
        (void)local_path_end;

        bool any_verified = false;
        for (const auto &c : candidates)
        {
            if (c.verified_frontier) { any_verified = true; break; }
        }
        if (!any_verified)
        {
            std::lock_guard<std::mutex> lk(selected_frontier_plane_mutex_);
            has_selected_frontier_plane_ = false;
            selected_frontier_plane_normal_.setZero();
            selected_frontier_plane_point_.setZero();
            selected_frontier_plane_width_ = 0.0;
            selected_frontier_plane_corridor_idx_ = -1;
            selected_frontier_plane_face_idx_ = -1;
            selected_frontier_plane_bfs_steps_ = -1;
            return false;
        }

        auto nearestStepsOnPath = [&](const Eigen::Vector3d &q,
                                      int &out_steps,
                                      Eigen::Vector3d &out_tangent,
                                      double &out_lateral_dist)->bool {
            double best_d2 = std::numeric_limits<double>::infinity();
            double best_exact_steps = -1.0;
            Eigen::Vector3d best_tangent(1.0, 0.0, 0.0);
            for (size_t i = 1; i < path_ptr->size(); ++i)
            {
                Eigen::Vector3d A = (*path_ptr)[i - 1]; A.z() = 0.0;
                Eigen::Vector3d B = (*path_ptr)[i];     B.z() = 0.0;
                Eigen::Vector3d P = q;                  P.z() = 0.0;
                
                Eigen::Vector3d AB = B - A;
                double len2 = AB.squaredNorm();
                double t = 0.0;
                if (len2 > 1e-6) {
                    t = (P - A).dot(AB) / len2;
                    t = std::max(0.0, std::min(1.0, t));
                }
                Eigen::Vector3d proj = A + t * AB;
                double d2 = (P - proj).squaredNorm();
                
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best_exact_steps = steps_acc[i - 1] + t * (steps_acc[i] - steps_acc[i - 1]);
                    if (len2 > 1e-9)
                    {
                        best_tangent = AB.normalized();
                    }
                }
            }
            // Allow up to 10m transversely to cover wide corridors
            const double accept_r_seg = 10.0; 
            if (!std::isfinite(best_d2) || best_d2 > accept_r_seg * accept_r_seg)
            {
                return false;
            }
            out_steps = (int)std::round(best_exact_steps);
            out_tangent = best_tangent;
            out_lateral_dist = std::sqrt(std::max(0.0, best_d2));
            return true;
        };

        PlaneCandidate best;
        int best_steps = -1;
        double best_width = -1.0;
        double best_score = -std::numeric_limits<double>::infinity();
        bool ok = false;

        const double eps = std::max(0.05, cellS * 0.5);
        const double push = config.dilateRadius + eps;
        const double offsets[5] = {0.0, eps, -eps, push, -push};

        // Forward ray at the END of the local path (用户要求：使用 local path 的终点作为射线原点).
        // This handles U-turns by testing what lies ahead of the path tip.
        Eigen::Vector3d ray_O = (*path_ptr).back();
        ray_O.z() = 0.0;
        Eigen::Vector3d ray_V = Eigen::Vector3d(1.0, 0.0, 0.0);
        if (path_ptr->size() >= 2)
        {
            const size_t kmax = std::min((size_t)5, path_ptr->size() - 1);
            for (size_t k = 1; k <= kmax; ++k)
            {
                Eigen::Vector3d dv = (*path_ptr).back() - (*path_ptr)[path_ptr->size() - 1 - k];
                dv.z() = 0.0;
                if (dv.norm() > 1e-3)
                {
                    ray_V = dv.normalized();
                    break;
                }
            }
        }
        ROS_WARN("Local path end ray: O=(%.3f, %.3f, %.3f) V=(%.6f, %.6f, %.6f)", ray_O.x(), ray_O.y(), ray_O.z(), ray_V.x(), ray_V.y(), ray_V.z());

        Eigen::Vector3d guide_dir = ray_V;
        {
            std::lock_guard<std::mutex> lk(status_mutex_);
            Eigen::Vector3d goal_vec = latest_goal_ - local_path_end;
            goal_vec.z() = 0.0;
            if (goal_vec.head<2>().norm() > 1e-6)
            {
                const Eigen::Vector3d goal_dir = goal_vec.normalized();
                const double agree = goal_dir.head<2>().dot(ray_V.head<2>());
                // If goal direction disagrees with local path tip direction, trust local tip to avoid backtracking.
                if (agree > 0.2)
                {
                    Eigen::Vector3d blend = 0.5 * ray_V + 0.5 * goal_dir;
                    if (blend.head<2>().norm() > 1e-6)
                    {
                        guide_dir = blend.normalized();
                    }
                }
            }
        }
        ROS_INFO("Frontier guide direction: G=(%.6f, %.6f, %.6f)", guide_dir.x(), guide_dir.y(), guide_dir.z());

        auto rayHitsPlaneSegmentXY = [&](const PlaneCandidate &cand)->bool {
            Eigen::Vector2d n(cand.normal_unit.x(), cand.normal_unit.y());
            const double n_norm = n.norm();
            if (!std::isfinite(n_norm) || n_norm < 1e-6)
            {
                // Horizontal-ish plane: cannot do a meaningful XY intersection test.
                return true;
            }
            n /= n_norm;
            const Eigen::Vector2d u(-n.y(), n.x()); // along-plane direction (unit)

            const Eigen::Vector2d O(ray_O.x(), ray_O.y());
            const Eigen::Vector2d v(ray_V.x(), ray_V.y());
            const Eigen::Vector2d P(cand.point_on_plane.x(), cand.point_on_plane.y());

            const double denom = n.dot(v);
            const double parallel_eps = 1e-3;
            if (std::abs(denom) < parallel_eps)
            {
                // Ray is (nearly) parallel to the plane line in XY: treat as no hit.
                return false;
            }

            // Ray-line intersection: n·(O + t v - P) = 0 -> t = n·(P - O) / n·v
            const double t = n.dot(P - O) / denom;
            const double t_allow = 0.5; // small tolerance (m) to handle numerical / discretization effects
            if (t < -t_allow) return false;

            const Eigen::Vector2d X = O + t * v;

            // Segment constraint using plane width along u.
            const double s = u.dot(X - P);
            const double s_margin = 0.2; // keep tight; width already accounts for corridor geometry
            if (std::abs(s) > cand.width * 0.5 + s_margin) return false;

            return true;
        };

        for (const auto &cand : candidates)
        {
            if (!cand.verified_frontier) continue;

            Eigen::Vector2d n2(cand.normal_unit.x(), cand.normal_unit.y());
            const double n2n = n2.norm();
            if (!std::isfinite(n2n) || n2n < 1e-6) continue;
            n2 /= n2n;

            const Eigen::Vector2d dOP(cand.point_on_plane.x() - ray_O.x(), cand.point_on_plane.y() - ray_O.y());
            const double along_ray = dOP.dot(ray_V.head<2>());
            const double front_gate_m = -0.2;
            if (along_ray < front_gate_m)
            {
                ROS_INFO("Frontier reject by front gate: corridor=%d face=%d along=%.3f gate=%.3f",
                         cand.corridor_idx, cand.face_idx, along_ray, front_gate_m);
                continue;
            }

            const bool ray_hit = rayHitsPlaneSegmentXY(cand);

            int cand_best_steps = -1;
            double cand_best_dir_align = -1.0;
            double cand_best_lateral = 1e9;
            for (int oi = 0; oi < 5; ++oi)
            {
                const Eigen::Vector3d q = cand.point_on_plane + offsets[oi] * cand.normal_unit;
                int steps_q = -1;
                Eigen::Vector3d tan_q(1.0, 0.0, 0.0);
                double lat_q = 1e9;
                if (!nearestStepsOnPath(q, steps_q, tan_q, lat_q)) continue;

                Eigen::Vector2d t2(tan_q.x(), tan_q.y());
                double dir_align = -1.0;
                if (t2.norm() > 1e-6)
                {
                    t2.normalize();
                    dir_align = t2.dot(n2);
                }

                if (steps_q > cand_best_steps ||
                    (steps_q == cand_best_steps && dir_align > cand_best_dir_align) ||
                    (steps_q == cand_best_steps && std::fabs(dir_align - cand_best_dir_align) < 1e-9 && lat_q < cand_best_lateral))
                {
                    cand_best_steps = steps_q;
                    cand_best_dir_align = dir_align;
                    cand_best_lateral = lat_q;
                }
            }

            // Fallback: if no sample is close to the local path, estimate its progress by extending from
            // the local path start along the general path direction.
            if (cand_best_steps < 0)
            {
                const Eigen::Vector3d start_pt = (*path_ptr).front();
                const Eigen::Vector3d end_pt = (*path_ptr).back();
                Eigen::Vector3d dir = end_pt - start_pt; dir.z() = 0.0;
                if (dir.norm() > 1e-6) dir.normalize();
                else dir = Eigen::Vector3d(1.0, 0.0, 0.0);
                
                const Eigen::Vector3d plane_pt = cand.point_on_plane;
                double proj = dir.dot(plane_pt - start_pt); // true distance along path vector
                cand_best_steps = (cellS > 1e-9) ? (int)std::floor(proj / cellS) : 0;
                cand_best_dir_align = dir.head<2>().dot(n2);
                cand_best_lateral = 0.0;
            }

            // Reject candidates whose projected samples are too far from the local path centerline.
            // This prevents large-offset side walls from dominating only because of large step counts.
            const double lateral_gate = std::max(2.5, 0.40 * cand.width + 0.9);
            if (cand_best_lateral > lateral_gate)
            {
                ROS_INFO("Frontier reject by lateral gate: corridor=%d face=%d lateral=%.3f gate=%.3f",
                         cand.corridor_idx, cand.face_idx, cand_best_lateral, lateral_gate);
                continue;
            }

            const double goal_align = guide_dir.head<2>().dot(n2);
            const double score =
                static_cast<double>(cand_best_steps) +
                3.0 * goal_align +
                2.0 * cand_best_dir_align -
                0.7 * cand_best_lateral +
                (ray_hit ? 0.5 : -0.5);

            ROS_INFO("Frontier score: corridor=%d face=%d steps=%d goal_align=%.3f dir_align=%.3f lat=%.3f ray_hit=%d score=%.3f",
                     cand.corridor_idx, cand.face_idx,
                     cand_best_steps, goal_align, cand_best_dir_align,
                     cand_best_lateral, ray_hit ? 1 : 0, score);

            if (score > best_score ||
                (std::fabs(score - best_score) < 1e-9 && cand_best_steps > best_steps) ||
                (std::fabs(score - best_score) < 1e-9 && cand_best_steps == best_steps && cand.width > best_width))
            {
                best = cand;
                best_steps = cand_best_steps;
                best_width = cand.width;
                best_score = score;
                ok = true;
            }
        }

        {
            std::lock_guard<std::mutex> lk(selected_frontier_plane_mutex_);
            if (ok)
            {
                has_selected_frontier_plane_ = true;
                selected_frontier_plane_normal_ = best.normal_unit;
                selected_frontier_plane_point_ = best.point_on_plane;
                selected_frontier_plane_width_ = best.width;
                selected_frontier_plane_corridor_idx_ = best.corridor_idx;
                selected_frontier_plane_face_idx_ = best.face_idx;
                selected_frontier_plane_bfs_steps_ = best_steps;
            }
            else
            {
                has_selected_frontier_plane_ = false;
                selected_frontier_plane_normal_.setZero();
                selected_frontier_plane_point_.setZero();
                selected_frontier_plane_width_ = 0.0;
                selected_frontier_plane_corridor_idx_ = -1;
                selected_frontier_plane_face_idx_ = -1;
                selected_frontier_plane_bfs_steps_ = -1;
            }
        }

        if (ok)
        {
            Eigen::Vector3d odom_pos(odom_local.pose.pose.position.x,
                                     odom_local.pose.pose.position.y,
                                     odom_local.pose.pose.position.z);
            Eigen::Vector3d n = best.normal_unit;
            double d = -(n.dot(best.point_on_plane));
            double dist_m = std::fabs(n.dot(odom_pos) + d);
            ROS_INFO("Selected verified frontier plane by A*: corridor=%d face=%d n=(%.3f, %.3f, %.3f) d=%.3f width=%.3f dist=%.3f steps=%d",
                     best.corridor_idx, best.face_idx,
                     n.x(), n.y(), n.z(), d, best.width, dist_m, best_steps);
        }
        else
        {
            ROS_INFO("No selectable verified frontier plane by A* local path");
        }
        return ok;
    }

    inline std::shared_ptr<PlaneCandidate> selectBestUnverifiedVerticalCollisionPlane(const std::vector<PlaneCandidate> &candidates,
                                                                                      const nav_msgs::Odometry &odom_local,
                                                                                      const double max_ang_deg = 10.0,
                                                                                      const double min_dist_m = 2.0,
                                                                                      const double max_dist_m = 6.0,
                                                                                      const double min_speed_mps = 0.10)
    {
        // Select among vertical candidates with verified_frontier==false.
        // Rule: angle(v_dir, n_xy) <= max_ang_deg AND (yaw aligns with v_dir or -v_dir within max_ang_deg)
        // AND Euclidean distance >= min_dist_m; then choose the max width.

        Eigen::Vector3d pos(odom_local.pose.pose.position.x,
                            odom_local.pose.pose.position.y,
                            odom_local.pose.pose.position.z);

        const auto &q = odom_local.pose.pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        Eigen::Vector2d yaw_dir(std::cos(yaw), std::sin(yaw));
        if (!yaw_dir.allFinite() || yaw_dir.norm() < 1e-9) yaw_dir = Eigen::Vector2d(1.0, 0.0);
        yaw_dir.normalize();

        Eigen::Vector2d v_dir(odom_local.twist.twist.linear.x, odom_local.twist.twist.linear.y);
        const double v_norm = v_dir.norm();
        const bool has_vel_dir = std::isfinite(v_norm) && v_norm >= min_speed_mps;
        if (has_vel_dir)
        {
            v_dir /= v_norm;
        }
        else
        {
            // if no clear velocity direction, use yaw directly
            v_dir = yaw_dir;
        }

        const double cos_th = std::cos(max_ang_deg * M_PI / 180.0);
        const double cos_th2 = cos_th * cos_th;

        std::shared_ptr<PlaneCandidate> best;
        double best_width = -1.0;
        double best_dist = 0.0;

        for (const auto &cand : candidates)
        {
            if (!cand.vertical) continue;
            if (cand.verified_frontier) continue; // we only handle unverified here

            Eigen::Vector2d n_xy(cand.normal_unit.x(), cand.normal_unit.y());
            const double n_norm = n_xy.norm();
            if (!std::isfinite(n_norm) || n_norm < 1e-6) continue;
            n_xy /= n_norm;

            // v_dir vs plane normal: accept parallel up to threshold (ignore sign of normal)
            const double dot_vn = v_dir.dot(n_xy);
            if (!(dot_vn >= cos_th)) continue;

            // yaw vs velocity direction: accept yaw ~ v_dir OR yaw ~ -v_dir (reverse flight)
            // Use squared dot to avoid sign, but still ensure close to either direction.
            const double dot_yv = yaw_dir.dot(v_dir);
            if (!(dot_yv * dot_yv >= cos_th2)) continue;

            const double dist_m = std::fabs(cand.normal_unit.dot(pos - cand.point_on_plane));
            if (!std::isfinite(dist_m) || dist_m < min_dist_m || dist_m > max_dist_m) continue;

            const double w = cand.width;
            if (!std::isfinite(w)) continue;
            if (w > best_width)
            {
                best_width = w;
                best_dist = dist_m;
                best = std::make_shared<PlaneCandidate>(cand);
            }
        }

        (void)best_dist; // currently only used for caching/logging
        return best;
    }

    inline bool updateSelectedUnverifiedCollisionPlaneByKinematics(const std::vector<PlaneCandidate> &candidates)
    {
        nav_msgs::Odometry odom_local;
        {
            std::lock_guard<std::mutex> lk(odom_mutex_);
            odom_local = odom;
        }
        
        // Debug: print incoming candidates for unverified-collision selection
        // for (size_t i = 0; i < candidates.size(); ++i)
        // {
        //     const auto &c = candidates[i];
        //     const Eigen::Vector3d &n = c.normal_unit;
        //     const Eigen::Vector3d &pt = c.point_on_plane;
        //     ROS_INFO("[updateSelectedUnverifiedCollisionPlane] Candidate[%zu]: corridor=%d face=%d vertical=%d verified=%d normal=(%.3f,%.3f,%.3f) point=(%.3f,%.3f,%.3f) width=%.3f",
        //              i, c.corridor_idx, c.face_idx, c.vertical ? 1 : 0, c.verified_frontier ? 1 : 0,
        //              n.x(), n.y(), n.z(), pt.x(), pt.y(), pt.z(), c.width);
        // }

        auto best = selectBestUnverifiedVerticalCollisionPlane(candidates, odom_local, 10.0, 2.0, 6.0, 0.10);
        double dist_m = 0.0;
        if (best)
        {
            Eigen::Vector3d pos(odom_local.pose.pose.position.x,
                                odom_local.pose.pose.position.y,
                                odom_local.pose.pose.position.z);
            dist_m = (best->point_on_plane - pos).norm();
        }

        // center point (odom snapshot) for later semicircle sampling
        Eigen::Vector3d center(odom_local.pose.pose.position.x,
                               odom_local.pose.pose.position.y,
                               odom_local.pose.pose.position.z);

        {
            std::lock_guard<std::mutex> lk(selected_collision_plane_mutex_);
            if (best)
            {
                // Additional filter: check semicircle area in front of vehicle for obstacles
                bool semicircle_has_obstacle = false;
                if (glbMapPtr && glbMapPtr->ogmPtr)
                {
                    auto ogm = glbMapPtr->ogmPtr;
                    double cellS = ogm->getScale();
                    if (cellS <= 1e-9) cellS = this->config.gridResolution > 1e-9 ? this->config.gridResolution : 0.2;

                    Eigen::Vector3d plane_pt = best->point_on_plane;
                    Eigen::Vector2d dir(plane_pt.x() - center.x(), plane_pt.y() - center.y());
                    if (dir.norm() < 1e-6) dir = Eigen::Vector2d(1.0, 0.0);
                    dir.normalize();

                    const double radius = std::max(0.5, dist_m);
                    const int ang_steps = 18; // 180deg / 10deg samples
                    for (int ai = -ang_steps/2; ai <= ang_steps/2 && !semicircle_has_obstacle; ++ai)
                    {
                        const double ang = (static_cast<double>(ai) / static_cast<double>(ang_steps)) * M_PI; // -pi/2 .. +pi/2
                        const double c = std::cos(ang), s = std::sin(ang);
                        Eigen::Vector2d v(c * dir.x() - s * dir.y(), s * dir.x() + c * dir.y());
                        // sample radial steps from half-cell to radius
                        for (double r = cellS * 0.5; r <= radius; r += cellS)
                        {
                            Eigen::Vector3d sample(center.x() + v.x() * r,
                                                   center.y() + v.y() * r,
                                                   plane_pt.z());
                            Eigen::Vector3i idx = ogm->convertPosD2I(sample);
                            idx.z() = ogm->convertPosD2I(Eigen::Vector3d(sample.x(), sample.y(), plane_pt.z())).z();
                            try {
                                if (ogm->queryIdx(idx)) { semicircle_has_obstacle = true; break; }
                            } catch(...) { /* ignore query errors */ }
                        }
                    }
                }

                if (!semicircle_has_obstacle)
                {
                    // Reject this collision plane as no obstacle present in semicircle area
                    has_selected_collision_plane_ = false;
                    selected_collision_plane_normal_.setZero();
                    selected_collision_plane_point_.setZero();
                    selected_collision_plane_width_ = 0.0;
                    selected_collision_plane_distance_m_ = 0.0;
                    selected_collision_plane_corridor_idx_ = -1;
                    selected_collision_plane_face_idx_ = -1;
                    ROS_INFO("Rejected collision plane by semicircle-obstacle-filter: dist=%.3f", dist_m);
                }
                else
                {
                    has_selected_collision_plane_ = true;
                    selected_collision_plane_normal_ = best->normal_unit;
                    selected_collision_plane_point_ = best->point_on_plane;
                    selected_collision_plane_width_ = best->width;
                    selected_collision_plane_distance_m_ = dist_m;
                    selected_collision_plane_corridor_idx_ = best->corridor_idx;
                    selected_collision_plane_face_idx_ = best->face_idx;
                }
            }
            else
            {
                has_selected_collision_plane_ = false;
                selected_collision_plane_normal_.setZero();
                selected_collision_plane_point_.setZero();
                selected_collision_plane_width_ = 0.0;
                selected_collision_plane_distance_m_ = 0.0;
                selected_collision_plane_corridor_idx_ = -1;
                selected_collision_plane_face_idx_ = -1;
            }
        }

        if (best)
        {
            Eigen::Vector3d n = best->normal_unit;
            double d = -(n.dot(best->point_on_plane));
            ROS_INFO("Selected unverified collision plane: corridor=%d face=%d n=(%.3f, %.3f, %.3f) d=%.3f width=%.3f dist=%.3f",
                     best->corridor_idx, best->face_idx,
                     n.x(), n.y(), n.z(), d, best->width, dist_m);
            return true;
        }

        ROS_INFO("No unverified collision plane matched (return None)");
        return false;
    }

    inline std::vector<PlaneCandidate> pubCorridor(const std::vector<Eigen::Matrix<double, 6, -1>> &corridorSeq, bool publish_frontier = true)
    {
        std::vector<PlaneCandidate> candidates;
        if (!ros::ok()) {
            ROS_WARN("pubCorridor aborted: ROS not ok");
            return candidates;
        }
        static int list_cnt = 0;
        quadrotor_msgs::CorridorList cor_list;

        cor_list.corridor_type = quadrotor_msgs::CorridorList::CORRIDOR_TYPE_H;
        cor_list.corridor_cnt = corridorSeq.size();
        int cor_cnt = 0;
        std::ostringstream frontier_ss;
        int global_cor_idx = 0;
        for (auto iter = corridorSeq.begin(); iter != corridorSeq.end(); iter++)
        {
            quadrotor_msgs::Corridor cor;
            cor.corridor_type = quadrotor_msgs::Corridor::CORRIDOR_TYPE_H;
            cor.cnt = cor_cnt++;
            cor.size = iter->cols();

            // Precompute polytope vertices once (for face width computation)
            std::vector<Eigen::Vector3d> poly_vertices;
            {
                const int m = iter->cols();
                std::vector<Eigen::Vector3d> normals(m);
                std::vector<double> bs(m);
                for (int pi = 0; pi < m; ++pi) {
                    const Eigen::Vector3d n = iter->col(pi).head<3>();
                    const Eigen::Vector3d p = iter->col(pi).tail<3>();
                    normals[pi] = n;
                    bs[pi] = n.dot(p);
                }

                const double det_eps = 1e-9;
                const double inside_tol = 1e-6;
                const double dedup_tol = 1e-3;
                for (int a = 0; a < m; ++a) {
                    for (int b = a + 1; b < m; ++b) {
                        for (int c = b + 1; c < m; ++c) {
                            Eigen::Matrix3d A;
                            A.row(0) = normals[a].transpose();
                            A.row(1) = normals[b].transpose();
                            A.row(2) = normals[c].transpose();
                            const double det = A.determinant();
                            if (!std::isfinite(det) || std::fabs(det) < det_eps) continue;
                            const Eigen::Vector3d rhs(bs[a], bs[b], bs[c]);
                            const Eigen::Vector3d x = A.fullPivLu().solve(rhs);
                            if (!x.allFinite()) continue;

                            bool inside = true;
                            for (int k = 0; k < m; ++k) {
                                const double v = normals[k].dot(x);
                                if (!std::isfinite(v) || v > bs[k] + inside_tol) {
                                    inside = false;
                                    break;
                                }
                            }
                            if (!inside) continue;

                            bool is_new = true;
                            for (const auto &vtx : poly_vertices) {
                                if ((vtx - x).norm() < dedup_tol) {
                                    is_new = false;
                                    break;
                                }
                            }
                            if (is_new) poly_vertices.push_back(x);
                        }
                    }
                }
            }
            geometry_msgs::Vector3 vec;
            geometry_msgs::Point pt;
            for (int i = 0; i < iter->cols(); i++)
            {
                vec.x = iter->col(i).array().head(3)[0];
                vec.y = iter->col(i).array().head(3)[1];
                vec.z = iter->col(i).array().head(3)[2];

                pt.x = iter->col(i).array().tail(3)[0];
                pt.y = iter->col(i).array().tail(3)[1];
                pt.z = iter->col(i).array().tail(3)[2];

                cor.nom_vec_list.push_back(vec);
                cor.point_list.push_back(pt);

                // Detect vertical faces (ignore ceiling/floor) and then verify true frontier
                const double nz = vec.z;
                const double vertical_thresh = 0.5; // if |nz| < thresh consider vertical wall
                if (std::fabs(nz) < vertical_thresh)
                {
                    // normalize normal
                    double nx = vec.x, ny = vec.y, nnz = vec.z;
                    double nrm = std::sqrt(nx * nx + ny * ny + nnz * nnz);
                    if (nrm > 1e-6)
                    {
                        nx /= nrm; ny /= nrm; nnz /= nrm;
                        // representative point on plane
                        Eigen::Vector3d p(pt.x, pt.y, pt.z);

                        PlaneCandidate cand;
                        cand.corridor_idx = global_cor_idx;
                        cand.face_idx = i;
                        cand.vertical = true;
                        cand.verified_frontier = false;
                        cand.normal_unit = Eigen::Vector3d(nx, ny, nnz);
                        // compute face centroid from poly_vertices (fallback to p)
                        {
                            Eigen::Vector3d face_centroid = p;
                            if (!poly_vertices.empty()) {
                                const double b_plane = cand.normal_unit.dot(p);
                                const double face_tol = 5e-3;
                                std::vector<Eigen::Vector3d> face_pts;
                                face_pts.reserve(poly_vertices.size());
                                for (const auto &vtx : poly_vertices) {
                                    const double dist_plane = std::fabs(cand.normal_unit.dot(vtx) - b_plane);
                                    if (dist_plane <= face_tol) face_pts.push_back(vtx);
                                }
                                if (!face_pts.empty()) {
                                    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
                                    for (const auto &v : face_pts) mean += v;
                                    mean /= static_cast<double>(face_pts.size());
                                    const double diff = cand.normal_unit.dot(mean) - b_plane;
                                    mean -= diff * cand.normal_unit;
                                    face_centroid = mean;
                                }
                            }
                            cand.point_on_plane = face_centroid;
                        }
                        cand.d = -(cand.normal_unit.dot(cand.point_on_plane));

                        // width along a horizontal tangent inside the plane
                        {
                            const Eigen::Vector3d world_z(0.0, 0.0, 1.0);
                            Eigen::Vector3d t = world_z.cross(cand.normal_unit);
                            if (t.norm() < 1e-6) {
                                const Eigen::Vector3d world_x(1.0, 0.0, 0.0);
                                t = world_x.cross(cand.normal_unit);
                            }
                            const double tn = t.norm();
                            if (tn > 1e-9 && !poly_vertices.empty()) {
                                t /= tn;
                                const double b_plane = cand.normal_unit.dot(cand.point_on_plane);
                                const double face_tol = 5e-3;
                                double smin = 1e100, smax = -1e100;
                                int cnt_face = 0;
                                for (const auto &vtx : poly_vertices) {
                                    const double dist_plane = std::fabs(cand.normal_unit.dot(vtx) - b_plane);
                                    if (dist_plane <= face_tol) {
                                        const double s = t.dot(vtx);
                                        if (s < smin) smin = s;
                                        if (s > smax) smax = s;
                                        ++cnt_face;
                                    }
                                }
                                if (cnt_face >= 2 && std::isfinite(smin) && std::isfinite(smax) && smax >= smin) {
                                    cand.width = smax - smin;
                                } else {
                                    cand.width = 0.0;
                                }
                            } else {
                                cand.width = 0.0;
                            }
                        }

                        // push candidate; frontier verification will be handled by shared function
                        cand.verified_frontier = false;
                        candidates.push_back(cand);
                    }
                }
            }
            cor_list.corridor_list.push_back(cor);
            ++global_cor_idx;
        }
        cor_list.header.frame_id = "world";
        cor_list.header.seq = list_cnt++;
        cor_list.header.stamp = ros::Time::now();
        corridorPub.publish(cor_list);
        // If requested, mark verified frontiers using shared safeQuery logic and build frontier text
        if (publish_frontier) {
            markVerifiedFrontiersBySafeQuery(candidates);
            // rebuild frontier summary
            std::string frontier_text;
            std::ostringstream oss;
            for (const auto &c : candidates) {
                if (c.verified_frontier) {
                    const Eigen::Vector3d &n = c.normal_unit;
                    oss << "corridor=" << c.corridor_idx << " face=" << c.face_idx
                        << " plane: n=(" << n.x() << "," << n.y() << "," << n.z() << ") d=" << c.d
                        << " width=" << c.width << "\n";
                }
            }
            frontier_text = oss.str();
            if (!frontier_text.empty())
            {
                std::lock_guard<std::mutex> lk(frontier_pub_mutex_);
                if (frontier_text != last_frontier_text_)
                {
                    std_msgs::String fps;
                    fps.data = frontier_text;
                    frontierPlanePub.publish(fps);
                    last_frontier_text_ = frontier_text;
                }
                else
                {
                    ROS_DEBUG("Frontier text identical to last published, skipping duplicate publish");
                }
            }
        }

        // Debug: print all detected plane candidates for inspection
        // for (unsigned long ci = 0; ci < candidates.size(); ++ci)
        // {
        //     const auto &c = candidates[ci];
        //     const Eigen::Vector3d &n = c.normal_unit;
        //     const Eigen::Vector3d &pt = c.point_on_plane;
        //     ROS_INFO("PlaneCandidate[%lu]: corridor=%d face=%d vertical=%d verified=%d normal=(%.3f,%.3f,%.3f) point=(%.3f,%.3f,%.3f) width=%.3f",
        //              ci,
        //              c.corridor_idx,
        //              c.face_idx,
        //              c.vertical ? 1 : 0,
        //              c.verified_frontier ? 1 : 0,
        //              n.x(), n.y(), n.z(),
        //              pt.x(), pt.y(), pt.z(),
        //              c.width);
        // }

        return candidates;
    }

    inline bool corridor_generate(Eigen::Vector3d origin, std::vector<Eigen::Matrix<double, 6, -1>> &corridorSeq)
    {
        // Snapshot bounds to avoid races with BoundCallback
        Eigen::Vector3d bound_min_local, bound_max_local;
        {
            std::lock_guard<std::mutex> lk(bound_mutex_);
            bound_min_local = bound_min;
            bound_max_local = bound_max;
        }

        if (origin[0] < bound_min_local(0) || origin[0] > bound_max_local(0) ||
            origin[1] < bound_min_local(1) || origin[1] > bound_max_local(1))
        {
            ROS_INFO("Out of bound!");
            return false;
        }

        double forceHeight = config.expectedHeight[0] - 0.1;
        if (origin(2) < forceHeight)
        {
            ROS_INFO("Your click is too low in height: z = %f, force it to be %f", origin(2), forceHeight);
            origin(2) = forceHeight;
        }
        // precompute the tiangle mesh of an unit cube
        const double unitCubeTris[108] = {0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1,
                                          0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0,
                                          0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0};

        Eigen::Map<const Eigen::Matrix<double, 36, 3, Eigen::ColMajor>> unitCubeMesh(unitCubeTris);

        Eigen::Matrix<double, 6, 1> bound;
        std::vector<double> localSurface;
        const int halfWi = std::max((int)(config.localBoxHalfWidth / config.gridResolution), 1);
        const double halfW = config.gridResolution * (halfWi + 0.5);
        localSurface.reserve((2 * halfWi + 1) * (2 * halfWi + 1) * (2 * halfWi + 1) * 3);

        bound(0) = std::max(bound_min_local(0), origin(0) - halfW);
        bound(1) = std::min(bound_max_local(0), origin(0) + halfW);
        bound(2) = std::max(bound_min_local(1), origin(1) - halfW);
        bound(3) = std::min(bound_max_local(1), origin(1) + halfW);
        bound(4) = std::max(bound_min_local(2), origin(2) - halfW);
        bound(5) = std::min(bound_max_local(2), origin(2) + halfW);

        bound(4) = std::max(bound(4), config.expectedHeight[0]);
        bound(5) = std::min(bound(5), config.expectedHeight[1]);

        Eigen::Matrix3Xd cubeMesh(3, unitCubeMesh.rows());
        cubeMesh.row(0) = unitCubeMesh.col(0).transpose().array() * (bound(1) - bound(0)) + bound(0);
        cubeMesh.row(1) = unitCubeMesh.col(1).transpose().array() * (bound(3) - bound(2)) + bound(2);
        cubeMesh.row(2) = unitCubeMesh.col(2).transpose().array() * (bound(5) - bound(4)) + bound(4);

        localSurface.clear();
        glbMapPtr->ogmPtr->getSurfacePointsInBox(glbMapPtr->ogmPtr->convertPosD2I(origin), halfWi, localSurface);

        std::vector<double> localSurface2;
        localSurface2.reserve(localSurface.size());
        for (int i = 0; i < (int)(localSurface.size()) / 3; i++)
        {
            if (localSurface[i * 3 + 2] > config.expectedHeight[1] + 0.51 * config.gridResolution ||
                localSurface[i * 3 + 2] < config.expectedHeight[0] - 0.51 * config.gridResolution)
            {
                continue;
            }
            localSurface2.push_back(localSurface[i * 3]);
            localSurface2.push_back(localSurface[i * 3 + 1]);
            localSurface2.push_back(localSurface[i * 3 + 2]);
        }
        localSurface = localSurface2;

        Eigen::Matrix<double, 6, -1> hPolytope;
        Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> localSurfaceMat(&(localSurface[0]), 3, localSurface.size() / 3);

        int obstacleSize = localSurfaceMat.cols();
        Eigen::Matrix<double, 3, -1> obstacleMesh(3, obstacleSize * 36 + 36);
        for (int i = 0; i < obstacleSize; i++)
        {
            obstacleMesh.block<1, 36>(0, 36 * i) = (unitCubeMesh.col(0).transpose().array() - 0.5) * config.gridResolution + localSurfaceMat(0, i);
            obstacleMesh.block<1, 36>(1, 36 * i) = (unitCubeMesh.col(1).transpose().array() - 0.5) * config.gridResolution + localSurfaceMat(1, i);
            obstacleMesh.block<1, 36>(2, 36 * i) = (unitCubeMesh.col(2).transpose().array() - 0.5) * config.gridResolution + localSurfaceMat(2, i);
        }
        obstacleMesh.rightCols<36>() = cubeMesh;
        Eigen::Matrix<double, 3, -1> tempPoints(3, 0);
        firi::maximalVolInsPolytope(obstacleMesh, tempPoints, origin, hPolytope);

        std::vector<Eigen::Matrix<double, 6, -1>> hPolytopes;
        hPolytopes.push_back(hPolytope);

        corridorSeq.push_back(hPolytope);
        lastPolytope = hPolytope;

        return true;
    }

    inline void savecorridor()
    {
        std::string path = ros::package::getPath("intention_get_corridor");
        std::ofstream fout(path + config.corridorPath);
        Eigen::VectorXi numscol(onetimeCorridor.size());
        for (size_t i = 0; i < onetimeCorridor.size(); i++)
        {
            numscol(i) = onetimeCorridor[i].cols();
        }
        fout << onetimeCorridor.size() << std::endl;
        for (size_t i = 0; i < onetimeCorridor.size(); i++)
        {
            fout << "6 " << onetimeCorridor[i].cols() << std::endl;
            fout << onetimeCorridor[i] << std::endl;
        }
        fout.close();
        std::cout << "File Saved!!!" << std::endl;
    }

    // === 发布完整的碰撞轨迹数据给轨迹优化模块（使用 CollisionTrajectory 消息） ===
    inline void publishCompleteCollisionTrajectory()
    {
        if (!ros::ok()) {
            ROS_WARN("publishCompleteCollisionTrajectory aborted: ROS not ok");
            return;
        }
        quadrotor_msgs::CollisionTrajectory collision_traj_msg;
        collision_traj_msg.header.stamp = ros::Time::now();
        collision_traj_msg.header.frame_id = "world";

        {
            std::lock_guard<std::mutex> lk(this->last_collision_mutex_);
            if (this->has_last_collision_traj_ &&
                (!this->last_collision_traj_.collision_events.empty() ||
                 !this->last_collision_traj_.trajectory_points.empty())) {
                collision_trajectory_pub_.publish(this->last_collision_traj_);
                return;
            }
        }

        ROS_INFO("No cached collision trajectory available, publishing empty CollisionTrajectory");
        collision_trajectory_pub_.publish(collision_traj_msg);
    }

    // preset collision trajectory helper removed

    // === 新增：发布目标点数据 ===
    void publishTargetGoal(const std::vector<Eigen::Vector3d>& pathList)
    {
        if (pathList.empty()) {
            ROS_WARN("No path points to publish target goal.");
            return;
        }

        geometry_msgs::PoseStamped target_goal;
        target_goal.header.frame_id = "world";
        target_goal.header.stamp = ros::Time::now();
        target_goal.pose.position.x = pathList.back().x();
        target_goal.pose.position.y = pathList.back().y();
        target_goal.pose.position.z = pathList.back().z();
        target_goal.pose.orientation.x = 0.0;
        target_goal.pose.orientation.y = 0.0;
        target_goal.pose.orientation.z = 0.0;
        target_goal.pose.orientation.w = 1.0;

        targetPub.publish(target_goal);
        ROS_INFO("Published target goal at the last path point: (%.3f, %.3f, %.3f)",
                 target_goal.pose.position.x, target_goal.pose.position.y, target_goal.pose.position.z);
    }
};
#endif
