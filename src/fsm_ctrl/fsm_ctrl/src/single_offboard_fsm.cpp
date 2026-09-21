/**
 * @file single_offboard_fsm.cpp
 * @brief Basic PX4 offboard control for a single drone.
 */

#include <fsm_ctrl/single_offboard_fsm.hpp>
#include <fsm_ctrl/px4ctrl_hover.hpp>

namespace
{
constexpr double kControlRateHz = 50.0;
constexpr double kInitialHeight = 1.0;
constexpr uint16_t kUdpPort = 12001;
constexpr int kRcThreshold = 1500;

std::atomic<int> command{0};
bool is_landed = false;
int takeoff_channel = 0;

mavros_msgs::State current_state;
ros::Time state_received;
Eigen::Vector3d local_position = Eigen::Vector3d::Zero();
geometry_msgs::PoseStamped position_setpoint;
mavros_msgs::AttitudeTarget attitude_setpoint;

void SetPosition(double x, double y, double z)
{
    position_setpoint.pose.position.x = x;
    position_setpoint.pose.position.y = y;
    position_setpoint.pose.position.z = z;
}

void StateCallback(const mavros_msgs::State::ConstPtr &message)
{
    current_state = *message;
    state_received = ros::Time::now();
}

void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &message)
{
    local_position = Eigen::Vector3d(
        message->pose.position.x,
        message->pose.position.y,
        message->pose.position.z);
}

void RcCallback(const mavros_msgs::RCIn::ConstPtr &message)
{
    if (message->channels.size() <= 8)
    {
        ROS_WARN_THROTTLE(1.0, "RC input has no takeoff channel (channel 9)");
        return;
    }
    takeoff_channel = message->channels[8];
}

void ListenForUdpCommands(uint16_t port)
{
    const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0)
    {
        ROS_ERROR("Failed to create UDP command socket");
        return;
    }

    sockaddr_in server_address{};
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(port);
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(socket_fd,
             reinterpret_cast<sockaddr *>(&server_address),
             sizeof(server_address)) < 0)
    {
        ROS_ERROR(
            "Failed to bind UDP command socket on port %u",
            static_cast<unsigned int>(port));
        close(socket_fd);
        return;
    }

    while (ros::ok())
    {
        char receive_buffer[100]{};
        sockaddr_in client_address{};
        socklen_t client_address_length = sizeof(client_address);
        const ssize_t received_size = recvfrom(
            socket_fd,
            receive_buffer,
            sizeof(receive_buffer) - 1,
            0,
            reinterpret_cast<sockaddr *>(&client_address),
            &client_address_length);

        if (received_size < 0)
        {
            ROS_ERROR_THROTTLE(1.0, "Failed to receive UDP command");
            continue;
        }

        receive_buffer[received_size] = '\0';
        int received_command = 0;
        if (std::sscanf(receive_buffer, "%d", &received_command) != 1)
        {
            ROS_WARN_THROTTLE(1.0, "Ignored malformed UDP command");
            continue;
        }
        command.store(received_command, std::memory_order_relaxed);
    }

    close(socket_fd);
}

void RequestOffboardAndArm(
    ros::ServiceClient &set_mode_client,
    ros::ServiceClient &arming_client,
    mavros_msgs::SetMode &offboard_mode,
    mavros_msgs::CommandBool &arm_command,
    ros::Time &last_request)
{
    const ros::Time now = ros::Time::now();
    if (current_state.mode != "OFFBOARD" &&
        now - last_request > ros::Duration(5.0))
    {
        if (set_mode_client.call(offboard_mode) &&
            offboard_mode.response.mode_sent)
        {
            ROS_WARN("Mode Offboard!");
        }
        last_request = now;
    }
    else if (!current_state.armed &&
             now - last_request > ros::Duration(5.0))
    {
        if (arming_client.call(arm_command) && arm_command.response.success)
        {
            ROS_WARN("Mode Armed!");
        }
        last_request = now;
    }
}
}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_offboard_fsm");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    ros::NodeHandle controller_params(private_node, "px4ctrl");
    Px4CtrlHover hover_controller(controller_params);
    double hover_x, hover_y, hover_z, hover_yaw;
    private_node.param("hover_x", hover_x, 0.0);
    private_node.param("hover_y", hover_y, 0.0);
    private_node.param("hover_z", hover_z, 0.4);
    private_node.param("hover_yaw", hover_yaw, 0.0);
    if (!std::isfinite(hover_x) || !std::isfinite(hover_y) ||
        !std::isfinite(hover_z) || !std::isfinite(hover_yaw))
    {
        ROS_FATAL("Invalid cmd=3 hover position/yaw");
        return 1;
    }

    const ros::Publisher position_publisher =
        node.advertise<geometry_msgs::PoseStamped>(
            "/mavros/setpoint_position/local", 10);
    const ros::Publisher attitude_publisher =
        node.advertise<mavros_msgs::AttitudeTarget>(
            "/mavros/setpoint_raw/attitude", 10);

    const ros::Subscriber state_subscriber =
        node.subscribe<mavros_msgs::State>(
            "/mavros/state", 10, StateCallback);
    const ros::Subscriber position_subscriber =
        node.subscribe<geometry_msgs::PoseStamped>(
            "/mavros/local_position/pose", 10,
            [&](const geometry_msgs::PoseStamped::ConstPtr &msg) {
                PoseCallback(msg);
                hover_controller.poseCallback(msg);
            });
    const ros::Subscriber velocity_subscriber =
        node.subscribe("/mavros/local_position/velocity_local", 10,
                       &Px4CtrlHover::velocityCallback, &hover_controller);
    const ros::Subscriber imu_subscriber =
        node.subscribe("/mavros/imu/data", 10,
                       &Px4CtrlHover::imuCallback, &hover_controller);
    const ros::Publisher debug_publisher =
        node.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10);
    // PoseStamped exposes x/y/z as ordinary time series in PlotJuggler.
    const ros::Publisher desired_trajectory_publisher =
        node.advertise<geometry_msgs::PoseStamped>("/fsm_ctrl/desired_trajectory", 50);
    const ros::Subscriber rc_subscriber =
        node.subscribe<mavros_msgs::RCIn>(
            "/mavros/rc/in", 10, RcCallback);

    ros::ServiceClient arming_client =
        node.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client =
        node.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode offboard_mode;
    offboard_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_command;
    arm_command.request.value = true;

    SetPosition(0.0, 0.0, kInitialHeight);
    position_setpoint.pose.orientation.w = 1.0;

    std::thread(ListenForUdpCommands, kUdpPort).detach();

    ros::Rate rate(kControlRateHz);
    for (int i = 0; ros::ok() && i < 100; ++i)
    {
        position_publisher.publish(position_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();
    int trajectory_step = 0;
    bool px4ctrl_active = false;

    while (ros::ok())
    {
        ros::spinOnce();

        const int current_command = command.load(std::memory_order_relaxed);
        if (current_command != 3 && current_command != 5) px4ctrl_active = false;
        switch (current_command)
        {
        case 1:
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);

            attitude_setpoint.header.frame_id = "FCU";
            attitude_setpoint.type_mask =
                mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
            attitude_setpoint.body_rate.x = 0.0;
            attitude_setpoint.body_rate.y = 0.0;
            attitude_setpoint.body_rate.z = 0.0;
            attitude_setpoint.thrust = 0.02;
            attitude_publisher.publish(attitude_setpoint);
            break;

        case 2:
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);
            SetPosition(0.0, 0.0, 1.0);
            position_publisher.publish(position_setpoint);
            break;

        case 3:
        {
            const ros::Time now = ros::Time::now();
            const double state_age = (now - state_received).toSec();
            if (!current_state.connected || state_received.isZero() ||
                state_age < 0.0 || state_age >= 2.0 || !hover_controller.ready(now))
            {
                px4ctrl_active = false;
                ROS_WARN_THROTTLE(1.0, "cmd=3 waiting for fresh PX4 pose, world velocity, IMU and state");
                break;
            }
            if (!px4ctrl_active)
            {
                hover_controller.reset();
                px4ctrl_active = true;
                ROS_INFO("cmd=3: original catkin_ws px4ctrl hover at [%.2f, %.2f, %.2f]",
                         hover_x, hover_y, hover_z);
            }
            geometry_msgs::Pose hover_target;
            hover_target.position.x = hover_x;
            hover_target.position.y = hover_y;
            hover_target.position.z = hover_z;
            hover_target.orientation.z = std::sin(hover_yaw * 0.5);
            hover_target.orientation.w = std::cos(hover_yaw * 0.5);
            geometry_msgs::PoseStamped desired_trajectory_point;
            desired_trajectory_point.header.stamp = now;
            desired_trajectory_point.header.frame_id = "map";
            desired_trajectory_point.pose = hover_target;
            desired_trajectory_publisher.publish(desired_trajectory_point);
            if (!current_state.armed)
            {
                attitude_publisher.publish(hover_controller.idle(now));
            }
            else
            {
                quadrotor_msgs::Px4ctrlDebug debug;
                if (!hover_controller.calculate(hover_target, now,
                        current_state.mode == "OFFBOARD", attitude_setpoint, debug))
                {
                    ROS_ERROR_THROTTLE(1.0, "cmd=3 rejected invalid hover control output");
                    break;
                }
                attitude_publisher.publish(attitude_setpoint);
                debug_publisher.publish(debug);
            }
            // Retain baseline OFFBOARD/arming workflow; publish no position target in cmd=3.
            arm_command.request.value = true;
            RequestOffboardAndArm(set_mode_client, arming_client, offboard_mode,
                                  arm_command, last_request);
            break;
        }

        case 4:
            if (!is_landed)
            {
                SetPosition(local_position.x(), local_position.y(), 0.005);
                position_publisher.publish(position_setpoint);
                is_landed = std::abs(local_position.z() - 0.05) < 0.05;
            }
            else if (current_state.mode != "OFFBOARD" && current_state.armed)
            {
                arm_command.request.value = false;
                if (arming_client.call(arm_command) &&
                    arm_command.response.success)
                {
                    ROS_WARN("Mode Disarm!");
                }
            }
            break;

        case 5:
        {
            const ros::Time now = ros::Time::now();
            const double state_age = (now - state_received).toSec();
            if (!current_state.connected || state_received.isZero() ||
                state_age < 0.0 || state_age >= 2.0 || !hover_controller.ready(now))
            {
                px4ctrl_active = false;
                ROS_WARN_THROTTLE(1.0, "cmd=5 waiting for fresh PX4 pose, world velocity, IMU and state");
                break;
            }
            if (!px4ctrl_active)
            {
                hover_controller.reset();
                px4ctrl_active = true;
                ROS_INFO("cmd=5: original catkin_ws px4ctrl circle tracking");
            }

            // Keep the baseline circle: x=0.75sin(t), y=0.75cos(t)-0.75, z=1.
            // The baseline advances t by 0.02 every 50 Hz cycle, hence omega=1 rad/s.
            const double phase = 0.02 * trajectory_step;
            geometry_msgs::Pose target;
            target.position.x = 0.75 * std::sin(phase);
            target.position.y = 0.75 * std::cos(phase) - 0.75;
            target.position.z = 1.0;
            target.orientation.w = 1.0;
            geometry_msgs::Vector3 target_velocity;
            target_velocity.x = 0.75 * std::cos(phase);
            target_velocity.y = -0.75 * std::sin(phase);
            geometry_msgs::Vector3 target_acceleration;
            target_acceleration.x = -0.75 * std::sin(phase);
            target_acceleration.y = -0.75 * std::cos(phase);

            geometry_msgs::PoseStamped desired_trajectory_point;
            desired_trajectory_point.header.stamp = now;
            desired_trajectory_point.header.frame_id = "map";
            desired_trajectory_point.pose = target;
            desired_trajectory_publisher.publish(desired_trajectory_point);

            if (!current_state.armed)
            {
                attitude_publisher.publish(hover_controller.idle(now));
            }
            else
            {
                quadrotor_msgs::Px4ctrlDebug debug;
                if (!hover_controller.calculate(target, target_velocity, target_acceleration,
                        now, current_state.mode == "OFFBOARD", attitude_setpoint, debug))
                {
                    ROS_ERROR_THROTTLE(1.0, "cmd=5 rejected invalid circle control output");
                    break;
                }
                attitude_publisher.publish(attitude_setpoint);
                debug_publisher.publish(debug);
            }
            arm_command.request.value = true;
            RequestOffboardAndArm(set_mode_client, arming_client, offboard_mode,
                                  arm_command, last_request);
            trajectory_step = (trajectory_step + 1) % 315;
            break;
        }

        case 6:
        {
            double trajectory_x = 0.0;
            double trajectory_y = 0.0;
            if (trajectory_step < 200)
            {
                trajectory_x = 1.5 * 0.005 * trajectory_step;
            }
            else if (trajectory_step < 400)
            {
                trajectory_x = 1.5;
                trajectory_y = -1.5 * 0.005 * (trajectory_step - 200);
            }
            else if (trajectory_step < 600)
            {
                trajectory_x =
                    1.5 - 1.5 * 0.005 * (trajectory_step - 400);
                trajectory_y = -1.5;
            }
            else
            {
                trajectory_y =
                    1.5 * 0.005 * (trajectory_step - 600) - 1.5;
            }

            SetPosition(trajectory_x, trajectory_y, 1.0);
            position_publisher.publish(position_setpoint);
            trajectory_step = (trajectory_step + 1) % 800;
            break;
        }

        case 10:
        {
            // Horizontal figure-eight: x = 0.75 sin(t), y = 0.375 sin(2t).
            const double phase = 0.02 * trajectory_step;
            SetPosition(0.75 * std::sin(phase),
                        0.375 * std::sin(2.0 * phase), 1.0);
            position_publisher.publish(position_setpoint);
            trajectory_step = (trajectory_step + 1) % 315;
            break;
        }

        default:
            break;
        }

        rate.sleep();
    }

    return 0;
}
