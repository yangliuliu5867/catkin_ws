#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/ESCStatus.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <ros/assert.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#define COLOR_RAD "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_END "\033[0m"
#define CLEAR_SCREEN "\033[1;1H\33[2J"

sensor_msgs::Imu ImuData;
void ImuDataCb(const sensor_msgs::Imu::ConstPtr &msg) { ImuData = *msg; }

mavros_msgs::ESCStatus ESCStatus;
void ESCStatusCb(const mavros_msgs::ESCStatus::ConstPtr &msg) { ESCStatus = *msg; }

nav_msgs::Odometry Odometry;
void OdometryCb(const nav_msgs::Odometry::ConstPtr &msg) { Odometry = *msg; }

quadrotor_msgs::Px4ctrlDebug Px4ctrlDebug;
void Px4ctrlDebugCb(const quadrotor_msgs::Px4ctrlDebug::ConstPtr &msg) { Px4ctrlDebug = *msg; }

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "debug");
    ros::NodeHandle nh("~");

    ImuData.header.stamp = ESCStatus.header.stamp = Odometry.header.stamp = Px4ctrlDebug.header.stamp = ros::Time::now();

    ros::Subscriber ImuDataSub = nh.subscribe<sensor_msgs::Imu>("imu_data", 100, ImuDataCb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber ESCStatusSub = nh.subscribe<mavros_msgs::ESCStatus>("esc_status", 100, ESCStatusCb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber OdometrySub = nh.subscribe<nav_msgs::Odometry>("odom", 100, OdometryCb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber Px4ctrlDebugSub = nh.subscribe<quadrotor_msgs::Px4ctrlDebug>("debugPx4ctrl", 100, Px4ctrlDebugCb, ros::TransportHints().tcpNoDelay());

    ros::Duration(1.0).sleep();

    ros::Rate rate(10);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();
        ros::Time now = ros::Time::now();

        printf(CLEAR_SCREEN);
        printf("------------------------------------------------------------------\n");

        if (now - ImuData.header.stamp > ros::Duration(0.5)) {
            printf("%s[     ImuData]%s: lost for %lfs\n", COLOR_RAD, COLOR_END, (now - ImuData.header.stamp).toSec());
        } else {
            printf("%s[     ImuData]%s:", COLOR_GREEN, COLOR_END);
            printf(" q=[%+5.3lf(x), %+5.3lf(y), %+5.3lf(z), %+5.3lf(w)]\n", ImuData.orientation.x, ImuData.orientation.y, ImuData.orientation.z, ImuData.orientation.w);

            printf("%s[     ImuData]%s:", COLOR_GREEN, COLOR_END);
            printf(" a=[%+5.3lf(x), %+5.3lf(y), %+5.3lf(z)]\n", ImuData.linear_acceleration.x, ImuData.linear_acceleration.y, ImuData.linear_acceleration.z);

            printf("%s[     ImuData]%s:", COLOR_GREEN, COLOR_END);
            printf(" omega=[%+5.3lf(x), %+5.3lf(y), %+5.3lf(z)]\n", ImuData.angular_velocity.x, ImuData.angular_velocity.y, ImuData.angular_velocity.z);
        }

        if (now - Odometry.header.stamp > ros::Duration(0.5)) {
            printf("%s[    Odometry]%s: lost for %lfs\n", COLOR_RAD, COLOR_END, (now - Odometry.header.stamp).toSec());
        } else {
            printf("%s[    Odometry]%s:", COLOR_GREEN, COLOR_END);
            printf(" p=[%+5.3lf(x), %+5.3lf(y), %+5.3lf(z)]\n", Odometry.pose.pose.position.x, Odometry.pose.pose.position.y, Odometry.pose.pose.position.z);

            printf("%s[    Odometry]%s:", COLOR_GREEN, COLOR_END);
            printf(" v=[%+5.3lf(x), %+5.3lf(y), %+5.3lf(z)]\n", Odometry.twist.twist.linear.x, Odometry.twist.twist.linear.y, Odometry.twist.twist.linear.z);

            printf("%s[    Odometry]%s:", COLOR_GREEN, COLOR_END);
            printf(" q=[%+5.3lf(x), %+5.3lf(y), %+5.3lf(z), %+5.3lf(w)]\n", Odometry.pose.pose.orientation.x, Odometry.pose.pose.orientation.y, Odometry.pose.pose.orientation.z, Odometry.pose.pose.orientation.w);
        }

        if (now - ESCStatus.header.stamp > ros::Duration(0.5)) {
            printf("%s[   ESCStatus]%s: lost for %lfs\n", COLOR_RAD, COLOR_END, (now - ESCStatus.header.stamp).toSec());
        } else if (ESCStatus.esc_status.size() > 0) {
            printf("%s[   ESCStatus]%s:", COLOR_GREEN, COLOR_END);
            printf(" rpm=[");
            for (std::size_t i = 0; i < ESCStatus.esc_status.size(); ++i) {
                printf(" %5d(%ld),", ESCStatus.esc_status[i].rpm, i);
            }
            printf("]\n");
        }

        if (now - Px4ctrlDebug.header.stamp > ros::Duration(0.5)) {
            printf("%s[Px4ctrlDebug]%s: lost for %lfs\n", COLOR_RAD, COLOR_END, (now - Px4ctrlDebug.header.stamp).toSec());
        } else {
            printf("%s[Px4ctrlDebug]%s:", COLOR_GREEN, COLOR_END);
            printf(" des_p=[%+5.3lf(x), %+5.3lf(y), %+5.3lf(z)]\n", Px4ctrlDebug.des_p_x, Px4ctrlDebug.des_p_y, Px4ctrlDebug.des_p_z);

            printf("%s[Px4ctrlDebug]%s:", COLOR_GREEN, COLOR_END);
            printf(" des_v=[%+5.3lf(x), %+5.3lf(y), %+5.3lf(z)]\n", Px4ctrlDebug.des_v_x, Px4ctrlDebug.des_v_y, Px4ctrlDebug.des_v_z);

            printf("%s[Px4ctrlDebug]%s:", COLOR_GREEN, COLOR_END);
            printf(" des_a=[%+5.3lf(x), %+5.3lf(y), %+5.3lf(z)]\n", Px4ctrlDebug.des_a_x, Px4ctrlDebug.des_a_y, Px4ctrlDebug.des_a_z);

            printf("%s[Px4ctrlDebug]%s:", COLOR_GREEN, COLOR_END);
            printf(" des_q=[%+5.3lf(x), %+5.3lf(y), %+5.3lf(z), %+5.3lf(w)]\n", Px4ctrlDebug.des_q_x, Px4ctrlDebug.des_q_y, Px4ctrlDebug.des_q_z, Px4ctrlDebug.des_q_w);

            printf("%s[Px4ctrlDebug]%s:", COLOR_GREEN, COLOR_END);
            printf(" des_thr=%lf\n", Px4ctrlDebug.des_thr);

            printf("%s[Px4ctrlDebug]%s:", COLOR_GREEN, COLOR_END);
            printf(" thr2acc=%lf\n", Px4ctrlDebug.thr2acc);

            printf("%s[Px4ctrlDebug]%s:", COLOR_GREEN, COLOR_END);
            printf(" hover_percentage=%lf\n", Px4ctrlDebug.hover_percentage);
        }
    }

    return 0;
}
