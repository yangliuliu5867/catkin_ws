#include "traj_gen_in_corridor/traj_gen_in_corridor.hpp"
#include "trt_manager.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_gen_in_corridor_node");
    ros::NodeHandle nh_;
    ros::NodeHandle pnh("~");

    // Load the parameters for TRT engine paths (can be overridden via launch)
    std::string engine_collision_bs72, engine_collision_bs1, engine_student;
    bool use_cuda_graph = true;
    bool verbose_timing = false;
    pnh.param<std::string>("engine_collision_bs72", engine_collision_bs72, std::string("/home/liet/mpd_splines/small/best_model_multimat.engine"));
    pnh.param<std::string>("engine_collision_bs1", engine_collision_bs1, std::string("/home/liet/mpd_splines/small/best_model_multimat_bs1.engine"));
    pnh.param<std::string>("engine_student", engine_student, std::string("/home/liet/mpd_splines/small/student_distilled.engine"));
    pnh.param<bool>("use_cuda_graph", use_cuda_graph, true);
    pnh.param<bool>("verbose_timing", verbose_timing, false);

    // Initialize and warm up TRT models in this process so traj_gen_in_corridor can call them directly
    trt::TrtManager &mgr = trt::TrtManager::instance();
    ROS_INFO("[traj_gen_in_corridor_node] Initializing TrtManager and warming up models...");
    bool ok = mgr.init(engine_collision_bs72, engine_collision_bs1, engine_student, use_cuda_graph, verbose_timing);
    if (ok) {
        ROS_INFO("[traj_gen_in_corridor_node] TrtManager warmup completed.");
        ros::param::set("/trt/warmup_ready", true);
    } else {
        ROS_ERROR("[traj_gen_in_corridor_node] TrtManager warmup FAILED.");
        ros::param::set("/trt/warmup_ready", false);
    }

    // Load the parameters
    Config config;
    config.load(ros::NodeHandle("~"));

    // Create a planner the run the visualization at a certain rate
    GlobalPlanner traj_gen_in_corridor(config, nh_);

    // traj_gen_in_corridor.loadCorridor();

    ros::spin();

    return 0;
}
