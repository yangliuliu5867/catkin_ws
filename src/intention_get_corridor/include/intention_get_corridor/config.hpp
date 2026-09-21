#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <vector>

#include <ros/ros.h>

struct Config
{
    std::string infmapTopic;            
    std::string odomTopic;              
    std::string targetTopic;           
    std::string pointCloudPath;       
    std::string corridorPath;           
    double dilateRadius;               
    double gridResolution;              
    std::vector<double> r3Bound;       
    double localBoxHalfWidth;           
    std::vector<double> expectedHeight; 
    int outlierThreshold;            
    bool useLoadPCDFile;                
    double astar_weight;               
    int cnt_pos;                        
    std::vector<double> set_att;        
    std::vector<double> set_pos;        
    
    // 注意：已移除预设碰撞路径相关参数以避免冗余
    
    // Kinodynamic parameters
    double max_velocity;                // 最大速度约束
    double max_acceleration;            // 最大加速度约束  
    double goal_tolerance;              // 目标点容忍度

    // 碰撞参数（新）
    double friction;                    // 切向速度保留系数（0.0 - 1.0），默认 0.65
    double damping_ratio;               // 法向阻尼比（0.0 - 1.0），默认 0.1
    double max_collision_velocity; // 碰撞模型中允许的最大碰撞速度

    // SampleForward 权重参数（默认值与旧实现保持一致）
    double w_collision;
    double w_kin;
    double w_dir;
    double w_end;

    // RViz visualization switches
    bool visualizeAstarPath{true};

    // 默认构造函数
    Config() = default;
    
    // 带参数的构造函数
    Config(ros::NodeHandle &nh_priv)
    {
        load(nh_priv);
    }
    
    // load方法，保持向后兼容性
    void load(ros::NodeHandle nh_priv)
    {
        nh_priv.getParam("InfMapTopic", infmapTopic);
        nh_priv.getParam("OdomTopic", odomTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("PointCloudPath", pointCloudPath);
        nh_priv.getParam("CorridorPath", corridorPath);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("GridResolution", gridResolution);
        nh_priv.getParam("R3Bound", r3Bound);
        nh_priv.getParam("LocalBoxHalfWidth", localBoxHalfWidth);
        nh_priv.getParam("ExpectedHeight", expectedHeight);
        nh_priv.getParam("OutlierThreshold", outlierThreshold);
        nh_priv.getParam("PointCloudUsePCD", useLoadPCDFile);
        nh_priv.getParam("Astar_weight", astar_weight);
        nh_priv.getParam("SetPos", set_pos);
        nh_priv.getParam("SetAtt", set_att);
        
        // 预设碰撞路径配置已移除
        
        // Kinodynamic parameters
        nh_priv.getParam("MaxVelocity", max_velocity);
        nh_priv.getParam("MaxAcceleration", max_acceleration);
        nh_priv.getParam("GoalTolerance", goal_tolerance);
        
        // 碰撞参数：使用新的键 `friction` 和 `damping_ratio`
        // 如果没有提供则使用默认值（friction=0.65, damping_ratio=0.1）
        if (!nh_priv.getParam("CollisionModel/friction", friction)) {
            friction = 0.65;
        }
        if (!nh_priv.getParam("CollisionModel/damping_ratio", damping_ratio)) {
            damping_ratio = 0.1;
        }
        // 从命名空间 CollisionModel 读取 max_collision_velocity（兼容现有 yaml）
        nh_priv.getParam("CollisionModel/max_collision_velocity", max_collision_velocity);
        // 从 SampleForward/weights 读取权重参数（可选，保持默认值如果不存在）
        nh_priv.getParam("SampleForward/weights/w_collision", w_collision);
        nh_priv.getParam("SampleForward/weights/w_kin", w_kin);
        nh_priv.getParam("SampleForward/weights/w_dir", w_dir);
        nh_priv.getParam("SampleForward/weights/w_end", w_end);

        // A* path visualization in RViz (default: true)
        nh_priv.param("VisualizeAstarPath", visualizeAstarPath, true);
        
        cnt_pos = set_pos.size() / 3;
        if (int(set_pos.size()) != cnt_pos * 3 || int(set_att.size()) != cnt_pos * 4)
        {
            ROS_ERROR("SetPos or SetAtt parameter size mismatch!");
            cnt_pos = 0;
        }
    }
};

#endif