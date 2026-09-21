#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <vector>

#include <ros/ros.h>

struct Config
{
    std::string targetTopic, triggerTopic;
    std::string odomTopic;
    std::string trajTopic, visTrajTopic;
    double maxOmgRate;
    double maxVelRate;
    double maxAccRate;
    double maxTau, minTau;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    double relCostTol;
    double quadratureResolution;
    bool isDebug;
    // 新参数原生字段
    double friction;               // tangential retention multiplier (default 0.65)
    double damping_ratio;          // normal damping fraction (default 0.1)
    double max_collision_velocity; // 最大碰撞速度

    // 碰撞约束参数
    double max_position_offset;
    double position_constraint_weight;
    double min_normal_velocity;
    double max_normal_velocity;
    double velocity_constraint_weight;
    // trajectory start delay (seconds) added to message header to allow controller buffering
    double traj_start_delay;
    
    // Load all parameters specified by ROS script
    inline void load(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("TriggerTopic", triggerTopic);
        nh_priv.getParam("OdomTopic", odomTopic);
        nh_priv.getParam("TrajTopic", trajTopic);
        nh_priv.getParam("VisTrajTopic", visTrajTopic);
        nh_priv.getParam("MaxOmgRate", maxOmgRate);
        nh_priv.getParam("MaxVelRate", maxVelRate);
        nh_priv.getParam("MaxAccRate", maxAccRate);
        nh_priv.getParam("MaxTau", maxTau);
        nh_priv.getParam("MinTau", minTau);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("RelCostTol", relCostTol);
        nh_priv.getParam("QuadratureResolution", quadratureResolution);
        nh_priv.getParam("debug", isDebug);

        // 加载碰撞模型参数：优先读取新的 `friction` 和 `damping_ratio` 字段
        if (!nh_priv.getParam("CollisionModel/friction", friction)) {
            friction = 0.65;
            ROS_WARN("CollisionModel/friction not found, using default: %.2f", friction);
        }

        if (!nh_priv.getParam("CollisionModel/damping_ratio", damping_ratio)) {
            damping_ratio = 0.1;
            ROS_WARN("CollisionModel/damping_ratio not found, using default: %.2f", damping_ratio);
        }

        // 使用新的物理参数：只读取 friction / damping_ratio / max_collision_velocity
        if (!nh_priv.getParam("CollisionModel/max_collision_velocity", max_collision_velocity)) {
            max_collision_velocity = 2.0;
            ROS_WARN("CollisionModel/max_collision_velocity not found, using default: %.2f", max_collision_velocity);
        }

        // 加载碰撞约束参数
        loadCollisionConstraintParams(nh_priv);

        // 输出碰撞约束参数
        ROS_INFO("=== Collision Constraints Configuration ===");
        ROS_INFO("Position constraints: %s (weight: %.1f, max_offset: %.3f m)", 
                 "ENABLED", 
                 position_constraint_weight, max_position_offset);
        ROS_INFO("Velocity constraints: %s (weight: %.1f, range: [%.1f, %.1f] m/s)", 
                 "ENABLED", 
                 velocity_constraint_weight, min_normal_velocity, max_normal_velocity);
        ROS_INFO("==========================================");

        ROS_INFO("Collision model loaded: friction=%.2f, damping_ratio=%.2f, max_collision_velocity=%.1f", 
            friction, damping_ratio, max_collision_velocity);

        // 轨迹发布延迟（用于给控制器或接收端一点处理缓冲），默认 0.05s
        nh_priv.param("TrajStartDelay", traj_start_delay, 0.05);
        ROS_INFO("Trajectory start delay set to %.3f seconds", traj_start_delay);
    }

    // 从参数服务器读取碰撞约束参数
    void loadCollisionConstraintParams(const ros::NodeHandle& nh)
    {
        nh.param("CollisionConstraints/max_position_offset", max_position_offset, 0.3);
        nh.param("CollisionConstraints/position_constraint_weight", position_constraint_weight, 1.0);
        nh.param("CollisionConstraints/min_normal_velocity", min_normal_velocity, 1.5);
        nh.param("CollisionConstraints/max_normal_velocity", max_normal_velocity, 2.0);
        nh.param("CollisionConstraints/velocity_constraint_weight", velocity_constraint_weight, 5.0);
    }
};

#endif