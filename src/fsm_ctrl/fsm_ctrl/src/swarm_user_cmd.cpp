/**
 * @file    swarm_user_cmd
 * @brief   finite state machine user interaction node
 * @author  FLAG Lab, BIT
 * @version 2.1
 * @date    2024-07-18
 */

#include <fsm_ctrl/swarm_user_cmd.hpp>

using namespace std;



static int cmd = -1;
static Eigen::Vector3d pos_vision, pos_aft, pos_local;
static Eigen::Vector3d euler_vision, euler_aft, euler_local;


/**
 * @brief  fusion pose subscriber callback
 * @param  _msg: geometry_msgs::PoseStamped from FCU
 * @return NULL
 */
void Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_local = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    euler_local = QuatToEuler(q);
}


/**
 * @brief  odometry subscriber callback
 * @param  msg: geometry_msgs::PoseStamped from odometry
 * @return NULL
 */
void Odom_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_vision = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    euler_vision = QuatToEuler(q);
}


/**
 * @brief  lidar odometry subscriber callback
 * @param  msg: nav_msgs::Odometry from Point-LIO
 * @return NULL
 */
void Aft_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    pos_aft = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    euler_aft = QuatToEuler(q);
}



void UdpServer(const char* ip, const uint16_t cport, const int UAVID)
{
    // ROS_INFO("UDP %d", UAVID);
    string ss;
    geometry_msgs::PoseStamped offset;
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        ROS_ERROR("Network Error");
        return;
    }
    struct sockaddr_in addr_client;
    int len;
    memset(&addr_client, 0, sizeof(struct sockaddr_in));
    addr_client.sin_family = AF_INET;
    addr_client.sin_addr.s_addr = inet_addr(ip);
    addr_client.sin_port = htons(cport);
    len = sizeof(addr_client);
    switch(UAVID)
    {
        case 1:
        {
            offset.pose.position.x = 0.0;
            offset.pose.position.y = 0.0;
            offset.pose.position.z = 0.0;
            break;
        }
        case 2:
        {
            offset.pose.position.x = -0.3;
            offset.pose.position.y = 0.3;
            offset.pose.position.z = 0.0;
            break;
        }
        case 3:
        {
            offset.pose.position.x = -0.3;
            offset.pose.position.y = -0.3;
            offset.pose.position.z = 0.0;
            break;
        }
        case 4:
        {
            offset.pose.position.x = 0.3;
            offset.pose.position.y = -0.3;
            offset.pose.position.z = 0.0;
            break;
        }
        default:
            break;
    }
    int sdlen;
    int send_num;
    char send_buf[100];
    ros::Rate rate(10.0);
    while(ros::ok())
    {
        memset(&send_buf, 0, sizeof(send_buf));
        sdlen = 0;
        sprintf(send_buf + sdlen, "%d", cmd >= 10? 10:cmd);
        sdlen = int(strlen(send_buf));
        send_buf[sdlen] = ',';
        send_buf[sdlen + 1] = '\0';
        sdlen = int(strlen(send_buf));
        sprintf(send_buf + sdlen,"%.3lf", offset.pose.position.x);
        sdlen = int(strlen(send_buf));
        send_buf[sdlen] = ',';
        send_buf[sdlen + 1] = '\0';
        sdlen = int(strlen(send_buf));
        sprintf(send_buf + sdlen,"%.3lf", offset.pose.position.y);
        sdlen = int(strlen(send_buf));
        send_buf[sdlen] = ',';
        send_buf[sdlen + 1] = '\0';
        sdlen = int(strlen(send_buf));
        sprintf(send_buf + sdlen,"%.3lf", offset.pose.position.z);
        send_num = sendto(sock_fd, send_buf, ssize_t(strlen(send_buf)), 0, (struct sockaddr*)&addr_client, len);
        if(send_num < 0)
        {
            ROS_ERROR("Send Fail!, UAV = %d",UAVID);
            // perror("sendto error:");
            // exit(1);
        }
        // ROS_INFO("Current Pub: %s", send_buf);
        // cout << ssize_t(strlen(send_buf)) << endl;
        if(cmd == 10) {break;}
        rate.sleep();
    }
}


/**
 * @brief  enter user command
 * @param  NULL
 * @return NULL
 */
void UserCmd()
{
    cin >> cmd;
    cout << "\033[A";
    // switch(cmd)
    // {
    //     case 1: 
    //         ROS_INFO("Arm publish!");
    //         break;
    //     case 2:
    //         ROS_INFO("Disarm publish!");
    //         break;
    //     case 3:
    //         ROS_INFO("Takeoff publish!");
    //         break;
    //     case 4:
    //         ROS_INFO("Land publish!");
    //         break;
    // }
}


/**
 * @brief  user command thread
 * @param  NULL
 * @return NULL
 */
void CmdListener()
{
    ros::Rate rate(10.0);
    while(ros::ok())
    {
        UserCmd();
        if(cmd == 0) {break;}
        rate.sleep();
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_user_cmd");
    ros::NodeHandle nh("~");
    double pose_info_rate;
    nh.param("pose_info_rate", pose_info_rate, 10.0);
    if (pose_info_rate <= 0.0) pose_info_rate = 10.0;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Subscriber pose_suber = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 1, Pose_Callback);
    ros::Subscriber odom_suber = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/vision_pose/pose", 1, Odom_Callback);
    ros::Subscriber aft_suber = nh.subscribe<nav_msgs::Odometry>
        ("/aft_mapped_to_init", 1, Aft_Callback);
    
    new thread(&CmdListener);
    new thread(&UdpServer, "127.0.0.1", 12001, 1);
    // new std::thread(&UdpServer,"192.168.1.11",12001,1);
    // new std::thread(&UdpServer,"192.168.1.12",12001,2);
    // new std::thread(&UdpServer,"192.168.1.13",12001,3);
    // new std::thread(&UdpServer,"192.168.1.14",12001,4);
    
    for(int i = 0; i < 9; i++) {cout << endl;}
    
    ros::Rate rate(pose_info_rate);
    while(ros::ok())
    {
        for(int i = 0; i < 9; i++) {cout << "\033[A";}

        cout.setf(ios::fixed);             // 固定的浮点显示
        cout << setprecision(3);           // 固定显示精度为2位
        cout.setf(ios::left);              // 左对齐
        cout.setf(ios::showpoint);         // 强制显示小数点
        cout.setf(ios::showpos);           // 强制显示符号
        cout << "\r" << "\x1B[0K";
        cout  << ">>>>>>>>>>>>>>>>>>>>>>>>>>>- Pose Info -<<<<<<<<<<<<<<<<<<<<<<<<<<<"  << endl;
        cout << "\r" << "\x1B[0K";
        cout << "[VISIONPOSITION] x: " << pos_vision(0) << "(m)   y: " << pos_vision(1) << "(m)   z: " << pos_vision(2)
             << "(m)   yaw: " << euler_vision(2)*180.0/M_PI << "(deg)" << endl;
        cout << "\r" << "\x1B[0K";
        cout << "[AFT] x: " << pos_aft(0) << "(m)   y: " << pos_aft(1) << "(m)   z: " << pos_aft(2)
             << "(m)   yaw: " << euler_aft(2)*180.0/M_PI << "(deg)" << endl;
        cout << "\r" << "\x1B[0K";
        cout << "[LOCALPOSITION] x: " << pos_local(0) << "(m)   y: " << pos_local(1) << "(m)   z: " << pos_local(2)
             << "(m)   yaw: " << euler_local(2)*180.0/M_PI << "(deg)" << endl;
        cout << "\r" << "\x1B[0K";
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>- User Info -<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
        cout << "\r" << "\x1B[0K";
        cout << "1: Arm         2: Disarm      3: Takeoff     4: Land        5: Debug" << endl;
        cout << "\r" << "\x1B[0K";
        cout << "6:             7:             8:             9:             0: Exit" << endl;
        
        cout.unsetf(ios::showpos);
        if(cmd == -1)
        {
            cout << "\r" << "\x1B[0K";
            cout << "[CMD] " << endl;
        }
        else
        {
            cout << "\r" << "\x1B[0K";
            cout << "[CMD] " << cmd << endl;
        }
        cout << "\r" << "\x1B[0K";
        cout << "Enter User Command: " << endl;
        
        if(cmd == 0) {break;}
        rate.sleep();
    }
    return 0;
}
