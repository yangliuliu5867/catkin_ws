#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>

ros::ServiceClient set_FCU_mode_srv;

bool toggle_offboard_mode(bool on_off) {
    mavros_msgs::SetMode offb_set_mode;

    if (on_off) {
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent)) {
            ROS_ERROR("Enter OFFBOARD rejected by PX4!");
            return false;
        }
    } else {
        offb_set_mode.request.custom_mode = "MANUAL";
        if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent)) {
            ROS_ERROR("Exit OFFBOARD rejected by PX4!");
            return false;
        }
    }

    return true;
}

// 测试时注意不要装螺旋桨 且遥控器要保持开机状态
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_attitude_target");
    ros::NodeHandle nh("~");

    set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Publisher ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    ROS_INFO("Try to toggle OFFBOARD mode");
    while (!toggle_offboard_mode(true)) {
        ROS_INFO("Retry to toggle OFFBOARD mode");
    }

    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = std::string("FCU");
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    msg.orientation.x = 0;
    msg.orientation.y = 0;
    msg.orientation.z = 0;
    msg.orientation.w = 1;
    msg.thrust = 0;

    ROS_INFO("Send mavros_msgs::AttitudeTarget");
    ros::Rate r(500);
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();
        ctrl_FCU_pub.publish(msg);
    }

    return 0;
}
