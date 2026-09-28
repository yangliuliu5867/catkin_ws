/**
 * @file px4_estimator.cpp
 * @brief Forward Point-LIO odometry to PX4 vision input and the flight stack.
 */

#include <fsm_ctrl/px4_estimator.hpp>

void LidarCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    const Eigen::Quaterniond lidar_orientation(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    const Eigen::Quaterniond lidar_to_body(
        Eigen::AngleAxisd(
            lidar_to_body_pitch_deg * M_PI / 180.0,
            Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond body_orientation =
        (lidar_orientation * lidar_to_body).normalized();

    vision_pose.header.stamp = msg->header.stamp;
    vision_pose.header.frame_id = "map";
    vision_pose.pose.position = msg->pose.pose.position;
    vision_pose.pose.orientation.w = body_orientation.w();
    vision_pose.pose.orientation.x = body_orientation.x();
    vision_pose.pose.orientation.y = body_orientation.y();
    vision_pose.pose.orientation.z = body_orientation.z();
    vision_pub.publish(vision_pose);

    nav_msgs::Odometry selected_odom = *msg;
    selected_odom.header.frame_id = "world";
    selected_odom.child_frame_id = "base_link";
    selected_odom.pose.pose.orientation = vision_pose.pose.orientation;
    selected_odom_pub.publish(selected_odom);
}

void FcuPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_fcu = Eigen::Vector3d(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z);
    quat_fcu = Eigen::Quaterniond(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);
    euler_fcu = QuatToEuler(quat_fcu);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_estimator");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("lidar_to_body_pitch_deg", lidar_to_body_pitch_deg, -25.3);

    ros::Publisher ready_pub =
        nh.advertise<std_msgs::Bool>("/fsm_ctrl/ekf_ready", 1);
    vision_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    selected_odom_pub =
        pnh.advertise<nav_msgs::Odometry>("odom_out", 10);

    const ros::Subscriber lidar_sub =
        pnh.subscribe<nav_msgs::Odometry>("lidar_odom", 1, LidarCallback);
    const ros::Subscriber pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>(
            "/mavros/local_position/pose", 10, FcuPoseCallback);

    ros::Rate rate(100.0);
    while (ros::ok())
    {
        ros::spinOnce();

        const Eigen::Vector3d vision_position(
            vision_pose.pose.position.x,
            vision_pose.pose.position.y,
            vision_pose.pose.position.z);
        const Eigen::Vector3d vision_euler = QuatToEuler(
            vision_pose.pose.orientation.w,
            vision_pose.pose.orientation.x,
            vision_pose.pose.orientation.y,
            vision_pose.pose.orientation.z);
        ekf_ready.data =
            !vision_pose.header.stamp.isZero() &&
            std::abs(vision_position.x() - pos_fcu.x()) < 0.05 &&
            std::abs(vision_position.y() - pos_fcu.y()) < 0.05 &&
            std::abs(vision_position.z() - pos_fcu.z()) < 0.05 &&
            std::abs(vision_euler.z() - euler_fcu.z()) < M_PI / 20.0;
        ready_pub.publish(ekf_ready);

        rate.sleep();
    }

    return 0;
}
