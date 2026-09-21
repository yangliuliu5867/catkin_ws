#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <uav_utils/utils.h>

class TrajConfig {
   public:
    enum TrajType {
        Circle = 0,
    };

    struct {
        double x_limit, y_limit, z_limit, yaw_limit;
        double x_period, y_period, z_period, yaw_period;
        double x_phase0, y_phase0, z_phase0, yaw_phase0;
    } circle;

    int type;

    template<typename TName, typename TVal>
    void getParam(const ros::NodeHandle &nh, const TName &name, TVal &val) {
        if (nh.getParam(name, val) == false) {
            ROS_ERROR_STREAM("Read param: " << name << " failed.");
            ROS_BREAK();
        }
    };
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "px4traj");
    ros::NodeHandle nh("~");

    TrajConfig cfg;
    cfg.getParam(nh, "type", cfg.type);

    switch (cfg.type) {
        case TrajConfig::TrajType::Circle:
            cfg.getParam(nh, "circle/x_limit", cfg.circle.x_limit);
            cfg.getParam(nh, "circle/y_limit", cfg.circle.y_limit);
            cfg.getParam(nh, "circle/z_limit", cfg.circle.z_limit);
            cfg.getParam(nh, "circle/yaw_limit", cfg.circle.yaw_limit);

            cfg.getParam(nh, "circle/x_period", cfg.circle.x_period);
            cfg.getParam(nh, "circle/y_period", cfg.circle.y_period);
            cfg.getParam(nh, "circle/z_period", cfg.circle.z_period);
            cfg.getParam(nh, "circle/yaw_period", cfg.circle.yaw_period);

            cfg.getParam(nh, "circle/x_phase0", cfg.circle.x_phase0);
            cfg.getParam(nh, "circle/y_phase0", cfg.circle.y_phase0);
            cfg.getParam(nh, "circle/z_phase0", cfg.circle.z_phase0);
            cfg.getParam(nh, "circle/yaw_phase0", cfg.circle.yaw_phase0);
            break;
        default:
            ROS_ERROR_STREAM("Unknown traj type: " << cfg.type);
            return -1;
    }

    bool on_off = false;
    geometry_msgs::Pose pose;
    ros::Subscriber traj_start_trigger_pub = nh.subscribe<geometry_msgs::PoseStamped>("/traj_start_trigger", 100, [&](const geometry_msgs::PoseStamped::ConstPtr &msg) {
        pose = msg->pose;
        if (msg->header.frame_id == "start")
            on_off = true;
        else
            on_off = false;
    });

    ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 100);
    quadrotor_msgs::PositionCommand cmd;

    ros::Time t0 = ros::Time::now();

    ros::Rate r(100);
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();

        if (on_off) {
            cmd.header.frame_id = "world";
            cmd.header.seq++;
            cmd.header.stamp = ros::Time::now();

            double t = (cmd.header.stamp - t0).toSec();

            switch (cfg.type) {
                case TrajConfig::TrajType::Circle:
                    cmd.position.x = cfg.circle.x_limit * (std::sin(2.0 * M_PI / cfg.circle.x_period * t + cfg.circle.x_phase0));
                    cmd.position.y = cfg.circle.y_limit * (std::sin(2.0 * M_PI / cfg.circle.y_period * t + cfg.circle.y_phase0));
                    cmd.position.z = cfg.circle.z_limit * (std::sin(2.0 * M_PI / cfg.circle.z_period * t + cfg.circle.z_phase0));

                    cmd.velocity.x = cfg.circle.x_limit * 2.0 * M_PI / cfg.circle.x_period * std::cos(2.0 * M_PI / cfg.circle.x_period * t + cfg.circle.x_phase0);
                    cmd.velocity.y = cfg.circle.y_limit * 2.0 * M_PI / cfg.circle.y_period * std::cos(2.0 * M_PI / cfg.circle.y_period * t + cfg.circle.y_phase0);
                    cmd.velocity.z = cfg.circle.z_limit * 2.0 * M_PI / cfg.circle.z_period * std::cos(2.0 * M_PI / cfg.circle.z_period * t + cfg.circle.z_phase0);

                    cmd.acceleration.x = -2.0 * M_PI / cfg.circle.x_period * 2.0 * M_PI / cfg.circle.x_period * cmd.position.x;
                    cmd.acceleration.y = -2.0 * M_PI / cfg.circle.y_period * 2.0 * M_PI / cfg.circle.y_period * cmd.position.y;
                    cmd.acceleration.z = -2.0 * M_PI / cfg.circle.z_period * 2.0 * M_PI / cfg.circle.z_period * cmd.position.z;

                    cmd.jerk.x = 0.0;
                    cmd.jerk.y = 0.0;
                    cmd.jerk.z = 0.0;

                    cmd.yaw = cfg.circle.yaw_limit * std::sin(2.0 * M_PI / cfg.circle.yaw_period * t + cfg.circle.yaw_phase0);
                    cmd.yaw_dot = cfg.circle.yaw_limit * 2.0 * M_PI / cfg.circle.yaw_period * std::cos(2.0 * M_PI / cfg.circle.yaw_period * t + cfg.circle.yaw_phase0);

                    break;
                default:
                    ROS_ERROR_STREAM("Unknown traj type: " << cfg.type);
                    return -1;
            }

            cmd.position.x += pose.position.x;
            cmd.position.y += pose.position.y;
            cmd.position.z += pose.position.z;

            cmd.yaw = uav_utils::normalize_angle(cmd.yaw);

            cmd_pub.publish(cmd);
        }
    }

    return 0;
}
