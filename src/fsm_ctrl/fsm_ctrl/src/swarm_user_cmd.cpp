/**
 * Jiangyin-compatible terminal UI for localization display and FSM commands.
 */
#include <fsm_ctrl/swarm_user_cmd.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace
{
std::atomic<int> user_command{-1};
std::atomic<int> fsm_command{0};
std::atomic<bool> running{true};
std::mutex state_mutex;

Eigen::Vector3d pos_vision = Eigen::Vector3d::Zero();
Eigen::Vector3d pos_aft = Eigen::Vector3d::Zero();
Eigen::Vector3d pos_local = Eigen::Vector3d::Zero();
Eigen::Vector3d euler_vision = Eigen::Vector3d::Zero();
Eigen::Vector3d euler_aft = Eigen::Vector3d::Zero();
Eigen::Vector3d euler_local = Eigen::Vector3d::Zero();
mavros_msgs::State mavros_state;
bool has_vision = false;
bool has_aft = false;
bool has_local = false;
bool has_mavros_state = false;

Eigen::Vector3d quaternionToEuler(const geometry_msgs::Quaternion &q_msg)
{
    Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    return QuatToEuler(q.normalized());
}

void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(state_mutex);
    pos_local << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    euler_local = quaternionToEuler(msg->pose.orientation);
    has_local = true;
}

void visionPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(state_mutex);
    pos_vision << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    euler_vision = quaternionToEuler(msg->pose.orientation);
    has_vision = true;
}

void lidarOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(state_mutex);
    pos_aft << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    euler_aft = quaternionToEuler(msg->pose.pose.orientation);
    has_aft = true;
}

void mavrosStateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(state_mutex);
    mavros_state = *msg;
    has_mavros_state = true;
}

void fsmCommandCallback(const std_msgs::Int32::ConstPtr &msg)
{
    fsm_command.store(msg->data, std::memory_order_relaxed);
}

void commandInputThread()
{
    int value = -1;
    while (ros::ok() && running.load())
    {
        if (!(std::cin >> value))
        {
            running.store(false);
            break;
        }
        user_command.store(value, std::memory_order_relaxed);
        if (value == 0)
        {
            running.store(false);
            break;
        }
    }
}

void udpCommandThread(const std::string &ip, uint16_t port)
{
    const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0)
    {
        ROS_ERROR("swarm_user_cmd: cannot create UDP socket");
        running.store(false);
        return;
    }

    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_port = htons(port);
    destination.sin_addr.s_addr = inet_addr(ip.c_str());

    int last_sent = -999;
    ros::Rate rate(20.0);
    while (ros::ok() && running.load())
    {
        const int value = user_command.load(std::memory_order_relaxed);
        if (value >= 0)
        {
            // Keep the original Jiangyin payload format. The single FSM reads
            // the leading integer and ignores the formation offsets.
            const std::string payload = std::to_string(value) + ",0.000,0.000,0.000";
            if (sendto(socket_fd, payload.data(), payload.size(), 0,
                       reinterpret_cast<sockaddr *>(&destination), sizeof(destination)) < 0)
            {
                ROS_ERROR_THROTTLE(1.0, "swarm_user_cmd: UDP send failed");
            }
            if (value != last_sent)
            {
                ROS_INFO("swarm_user_cmd: send cmd %d to %s:%u", value, ip.c_str(), port);
                last_sent = value;
            }
        }
        rate.sleep();
    }
    close(socket_fd);
}

std::string poseLine(const char *name, bool ready,
                     const Eigen::Vector3d &position,
                     const Eigen::Vector3d &euler)
{
    std::ostringstream out;
    out << std::left << std::setw(18) << name;
    if (!ready)
    {
        out << "等待数据";
        return out.str();
    }
    out << std::fixed << std::showpos << std::setprecision(3)
        << "x=" << position.x() << "  y=" << position.y()
        << "  z=" << position.z() << " m  yaw="
        << euler.z() * 180.0 / M_PI << " deg";
    return out.str();
}
}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_user_cmd");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string local_pose_topic = "/mavros/local_position/pose";
    std::string vision_pose_topic = "/mavros/vision_pose/pose";
    std::string lidar_odom_topic = "/aft_mapped_to_init";
    std::string state_topic = "/mavros/state";
    std::string fsm_command_topic = "/fsm_ctrl/command";
    std::string udp_ip = "127.0.0.1";
    int udp_port = 12001;
    double display_rate = 5.0;
    pnh.param("local_pose_topic", local_pose_topic, local_pose_topic);
    pnh.param("vision_pose_topic", vision_pose_topic, vision_pose_topic);
    pnh.param("lidar_odom_topic", lidar_odom_topic, lidar_odom_topic);
    pnh.param("state_topic", state_topic, state_topic);
    pnh.param("fsm_command_topic", fsm_command_topic, fsm_command_topic);
    pnh.param("udp_ip", udp_ip, udp_ip);
    pnh.param("udp_port", udp_port, udp_port);
    pnh.param("pose_info_rate", display_rate, display_rate);

    const ros::Subscriber local_sub = nh.subscribe(local_pose_topic, 5, localPoseCallback);
    const ros::Subscriber vision_sub = nh.subscribe(vision_pose_topic, 5, visionPoseCallback);
    const ros::Subscriber lidar_sub = nh.subscribe(lidar_odom_topic, 5, lidarOdomCallback);
    const ros::Subscriber state_sub = nh.subscribe(state_topic, 5, mavrosStateCallback);
    const ros::Subscriber command_sub = nh.subscribe(fsm_command_topic, 5, fsmCommandCallback);
    (void)local_sub; (void)vision_sub; (void)lidar_sub; (void)state_sub; (void)command_sub;

    ros::AsyncSpinner spinner(2);
    spinner.start();
    std::thread input_thread(commandInputThread);
    // std::cin may remain blocked when roslaunch is interrupted. Detaching
    // preserves the original interactive behavior without hanging shutdown.
    input_thread.detach();
    std::thread udp_thread(udpCommandThread, udp_ip, static_cast<uint16_t>(udp_port));

    ros::Rate rate(std::max(1.0, display_rate));
    while (ros::ok() && running.load())
    {
        Eigen::Vector3d local_pos, vision_pos, aft_pos;
        Eigen::Vector3d local_euler, vision_euler, aft_euler;
        mavros_msgs::State state;
        bool local_ready, vision_ready, aft_ready, state_ready;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            local_pos = pos_local; vision_pos = pos_vision; aft_pos = pos_aft;
            local_euler = euler_local; vision_euler = euler_vision; aft_euler = euler_aft;
            state = mavros_state;
            local_ready = has_local; vision_ready = has_vision; aft_ready = has_aft;
            state_ready = has_mavros_state;
        }

        std::cout << "\033[2J\033[H"
                  << ">>>>>>>>>>>>>>>>>>> 定位与状态机交互 <<<<<<<<<<<<<<<<<<<<<\n"
                  << poseLine("Point-LIO 原始", aft_ready, aft_pos, aft_euler) << '\n'
                  << poseLine("送入 PX4 视觉位姿", vision_ready, vision_pos, vision_euler) << '\n'
                  << poseLine("PX4 Local Position", local_ready, local_pos, local_euler) << '\n'
                  << "MAVROS: ";
        if (state_ready)
        {
            std::cout << "connected=" << state.connected
                      << " armed=" << state.armed
                      << " mode=" << state.mode;
        }
        else
        {
            std::cout << "等待数据";
        }
        std::cout << "\nFSM 当前命令: " << fsm_command.load()
                  << "    最近输入: " << user_command.load() << "\n\n"
                  << "1: OFFBOARD/解锁预备   2: PX4位置1m   3: NMPC悬停\n"
                  << "4: 正常降落            5: NMPC规划跟踪  7: NMPC八字轨迹\n"
                  << "0: 退出并发送cmd0（飞行中不要退出）\n"
                  << "请输入命令后回车: " << std::flush;
        rate.sleep();
    }

    running.store(false);
    ros::shutdown();
    if (udp_thread.joinable()) udp_thread.join();
    return 0;
}
