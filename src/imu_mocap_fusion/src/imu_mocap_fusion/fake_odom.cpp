
/******************************************************************************
 * Copyright 2023 YYHAN YIN. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_mocap_fusion_fake");
    ros::NodeHandle nh("~");

    ros::Publisher imfPub = nh.advertise<nav_msgs::Odometry>("odom", 100);

    ros::Subscriber imuSub = nh.subscribe<sensor_msgs::Imu>("imu", 100, [&](const sensor_msgs::Imu::ConstPtr &msg) {
        static nav_msgs::Odometry odom;

        odom.header.stamp = msg->header.stamp;
        odom.header.seq++;
        odom.header.frame_id = std::string("world");
        odom.child_frame_id = std::string("base_link");

        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        odom.pose.pose.position.z = 0;

        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.linear.z = 0;

        odom.pose.pose.orientation = msg->orientation;

        odom.twist.twist.angular = msg->angular_velocity;

        imfPub.publish(odom);
    });

    ros::spin();

    return 0;
}
