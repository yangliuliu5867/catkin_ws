
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

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "mocap_fix");
    ros::NodeHandle nh("~");

    ros::Publisher mocapPub = nh.advertise<geometry_msgs::PoseStamped>("/vrpn_client_node/tx_uav/pose_fix", 100);
    ros::Subscriber mocapSub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/vrpn_client_node/tx_uav/pose",
        100,
        [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            geometry_msgs::PoseStamped msg_fix = *msg;

            msg_fix.pose.position.y = msg->pose.position.x;
            msg_fix.pose.position.z = msg->pose.position.y;
            msg_fix.pose.position.x = msg->pose.position.z;

            msg_fix.pose.orientation.y = msg->pose.orientation.x;
            msg_fix.pose.orientation.z = msg->pose.orientation.y;
            msg_fix.pose.orientation.x = msg->pose.orientation.z;

            mocapPub.publish(msg_fix);
        },
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    geometry_msgs::PoseStamped msg_new;

    ros::spin();

    return 0;
}
