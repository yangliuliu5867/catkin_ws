
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "mocap_fake");
    ros::NodeHandle nh("~");

    ros::Publisher mocapPub = nh.advertise<geometry_msgs::PoseStamped>("/vrpn_client_node/tx_uav/pose", 100);

    geometry_msgs::PoseStamped msg_new;

    msg_new.pose.position.x = 0;
    msg_new.pose.position.y = 0;
    msg_new.pose.position.z = 0;

    msg_new.pose.orientation.x = 0;
    msg_new.pose.orientation.y = 0;
    msg_new.pose.orientation.z = 0;
    msg_new.pose.orientation.w = 1;

    ros::Rate rate(60);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();

        mocapPub.publish(msg_new);
    }

    return 0;
}
