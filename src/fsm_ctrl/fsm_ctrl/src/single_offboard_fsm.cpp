/**
 * @file single_offboard_fsm.cpp
 * @brief Basic PX4 offboard control for a single drone.
 */

#include <fsm_ctrl/single_offboard_fsm.hpp>

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
mavros_msgs::ExtendedState current_extended_state;
bool has_extended_state = false;
ros::Time extended_state_stamp;
sensor_msgs::Imu latest_imu;
sensor_msgs::BatteryState latest_battery;
Eigen::Vector3d local_position = Eigen::Vector3d::Zero();
ros::Time local_pose_stamp;
Eigen::Vector3d local_velocity = Eigen::Vector3d::Zero();
ros::Time local_velocity_stamp;
Eigen::Quaterniond local_attitude = Eigen::Quaterniond::Identity();
quadrotor_msgs::PositionCommand planner_command;
quadrotor_msgs::PositionCommandArray planner_horizon;
ros::Time planner_command_stamp;
ros::Time planner_horizon_stamp;
bool has_local_pose = false;
bool has_local_velocity = false;
bool has_planner_command = false;
bool has_planner_horizon = false;
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
}

void ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &message)
{
    current_extended_state = *message;
    has_extended_state = true;
    extended_state_stamp = ros::Time::now();
}

void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &message)
{
    local_position = Eigen::Vector3d(
        message->pose.position.x,
        message->pose.position.y,
        message->pose.position.z);
    local_attitude = Eigen::Quaterniond(message->pose.orientation.w, message->pose.orientation.x, message->pose.orientation.y, message->pose.orientation.z);
    local_attitude.normalize();
    local_pose_stamp = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    has_local_pose = true;
}

void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &message)
{
    local_velocity = Eigen::Vector3d(message->twist.linear.x, message->twist.linear.y, message->twist.linear.z);
    local_velocity_stamp = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    has_local_velocity = true;
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr &message)
{
    latest_imu = *message;
}

void BatteryCallback(const sensor_msgs::BatteryState::ConstPtr &message)
{
    latest_battery = *message;
}

void PlannerCommandCallback(const quadrotor_msgs::PositionCommand::ConstPtr &message)
{
    planner_command = *message;
    planner_command_stamp = ros::Time::now();
    has_planner_command = true;
}

void PlannerHorizonCallback(const quadrotor_msgs::PositionCommandArray::ConstPtr &message)
{
    if (message->points.empty())
    {
        ROS_WARN_THROTTLE(1.0, "Ignored empty NMPC reference horizon");
        return;
    }
    planner_horizon = *message;
    planner_horizon_stamp = ros::Time::now();
    has_planner_horizon = true;
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

    std::string planner_command_topic = "/position_command";
    std::string planner_horizon_topic = "/position_command_horizon";
    std::string mission_mode = "single";
    private_node.param("planner_command_topic", planner_command_topic, planner_command_topic);
    private_node.param("planner_horizon_topic", planner_horizon_topic, planner_horizon_topic);
    node.param("/Mission/mode", mission_mode, mission_mode);

    const ros::Publisher position_publisher =
        node.advertise<geometry_msgs::PoseStamped>(
            "/mavros/setpoint_position/local", 10);
    // All body-rate commands on this branch use the MAVROS attitude plugin.
    // setpoint_raw clips negative thrust and cannot serve a 3D airframe.
    const ros::Publisher reverse_rate_publisher =
        node.advertise<geometry_msgs::TwistStamped>(
            "/mavros/setpoint_attitude/cmd_vel", 10);
    const ros::Publisher reverse_thrust_publisher =
        node.advertise<mavros_msgs::Thrust>(
            "/mavros/setpoint_attitude/thrust", 10);
    const ros::Publisher trajectory_reference_publisher =
        node.advertise<geometry_msgs::PoseStamped>(
            "/single_offboard_fsm/trajectory_reference", 10);
    const ros::Publisher trajectory_reference_velocity_publisher =
        node.advertise<geometry_msgs::TwistStamped>(
            "/single_offboard_fsm/trajectory_reference_velocity", 10);
    const ros::Publisher nmpc_state_publisher =
        node.advertise<fsm_ctrl::nmpc_state>("/nmpc_state", 10);
    const ros::Publisher control_debug_publisher =
        node.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10);
    const ros::Publisher command_state_publisher =
        node.advertise<std_msgs::Int32>("/fsm_ctrl/command", 10, true);
    const ros::Publisher planner_start_trigger_publisher =
        node.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 1, true);

    const ros::Subscriber state_subscriber =
        node.subscribe<mavros_msgs::State>(
            "/mavros/state", 10, StateCallback);
    const ros::Subscriber extended_state_subscriber =
        node.subscribe<mavros_msgs::ExtendedState>(
            "/mavros/extended_state", 10, ExtendedStateCallback);
    const ros::Subscriber position_subscriber =
        node.subscribe<geometry_msgs::PoseStamped>(
            "/mavros/local_position/pose", 10, PoseCallback);
    const ros::Subscriber velocity_subscriber =
        node.subscribe<geometry_msgs::TwistStamped>(
            "/mavros/local_position/velocity_local", 10, VelocityCallback);
    const ros::Subscriber planner_command_subscriber =
        node.subscribe<quadrotor_msgs::PositionCommand>(
            planner_command_topic, 10, PlannerCommandCallback,
            ros::TransportHints().tcpNoDelay());
    const ros::Subscriber planner_horizon_subscriber =
        node.subscribe<quadrotor_msgs::PositionCommandArray>(
            planner_horizon_topic, 5, PlannerHorizonCallback,
            ros::TransportHints().tcpNoDelay());
    const ros::Subscriber rc_subscriber =
        node.subscribe<mavros_msgs::RCIn>(
            "/mavros/rc/in", 10, RcCallback);
    const ros::Subscriber imu_subscriber =
        node.subscribe<sensor_msgs::Imu>(
            "/mavros/imu/data", 50, ImuCallback,
            ros::TransportHints().tcpNoDelay());
    const ros::Subscriber battery_subscriber =
        node.subscribe<sensor_msgs::BatteryState>(
            "/mavros/battery", 10, BatteryCallback,
            ros::TransportHints().tcpNoDelay());

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

    double qpx = 1.0, qpy = 1.0, qpz = 1.0, qvx = 1.0, qvy = 1.0, qvz = 1.0;
    double qqx = 1.0, qqy = 1.0, qqz = 1.0, rwx = 1.0, rwy = 1.0, rwz = 1.0;
    double rthrust = 1.0, hover_thrust = 0.196;
    double planner_command_timeout = 0.5;
    double nmpc_gravity = 9.8015;
    double hover_x = 0.0, hover_y = 0.0, hover_z = 1.4, hover_yaw = 0.0;
    bool enable_cmd8 = true;
    double roll_duration_sec = 1.2;
    double roll_rate_limit = 6.0;
    double roll_min_height = 0.8;
    double roll_max_entry_speed = 0.25;
    double roll_state_timeout_sec = 0.3;
    double inverted_land_speed = 0.20;
    bool enable_xy_integral = true;
    double xy_integral_gain = 0.08;
    double xy_integral_limit = 0.10;
    double xy_integral_leak = 0.01;
    double xy_integral_max_error = 0.30;
    bool ndob_enable = true;
    double ndob_bandwidth_x = 2.0, ndob_bandwidth_y = 2.0, ndob_bandwidth_z = 3.0;
    double ndob_limit_x = 3.0, ndob_limit_y = 3.0, ndob_limit_z = 6.0;
    private_node.param("nmpc_Qposx", qpx, qpx); private_node.param("nmpc_Qposy", qpy, qpy); private_node.param("nmpc_Qposz", qpz, qpz);
    private_node.param("nmpc_Qvelx", qvx, qvx); private_node.param("nmpc_Qvely", qvy, qvy); private_node.param("nmpc_Qvelz", qvz, qvz);
    private_node.param("nmpc_Qquatx", qqx, qqx); private_node.param("nmpc_Qquaty", qqy, qqy); private_node.param("nmpc_Qquatz", qqz, qqz);
    private_node.param("nmpc_Rwx", rwx, rwx); private_node.param("nmpc_Rwy", rwy, rwy); private_node.param("nmpc_Rwz", rwz, rwz);
    private_node.param("nmpc_RtotalF", rthrust, rthrust); private_node.param("nmpc_hover_thrust", hover_thrust, hover_thrust);
    private_node.param("planner_command_timeout", planner_command_timeout, planner_command_timeout);
    private_node.param("nmpc_gravity", nmpc_gravity, nmpc_gravity);
    private_node.param("nmpc_hover_x", hover_x, hover_x);
    private_node.param("nmpc_hover_y", hover_y, hover_y);
    private_node.param("nmpc_hover_z", hover_z, hover_z);
    private_node.param("nmpc_hover_yaw", hover_yaw, hover_yaw);
    private_node.param("enable_cmd8", enable_cmd8, enable_cmd8);
    private_node.param("roll_duration_sec", roll_duration_sec, roll_duration_sec);
    private_node.param("roll_rate_limit", roll_rate_limit, roll_rate_limit);
    private_node.param("roll_min_height", roll_min_height, roll_min_height);
    private_node.param("roll_max_entry_speed", roll_max_entry_speed, roll_max_entry_speed);
    private_node.param("roll_state_timeout_sec", roll_state_timeout_sec, roll_state_timeout_sec);
    private_node.param("inverted_land_speed", inverted_land_speed, inverted_land_speed);
    private_node.param("nmpc_enable_xy_integral", enable_xy_integral, enable_xy_integral);
    private_node.param("nmpc_xy_integral_gain", xy_integral_gain, xy_integral_gain);
    private_node.param("nmpc_xy_integral_limit", xy_integral_limit, xy_integral_limit);
    private_node.param("nmpc_xy_integral_leak", xy_integral_leak, xy_integral_leak);
    private_node.param("nmpc_xy_integral_max_error", xy_integral_max_error, xy_integral_max_error);
    private_node.param("nmpc_ndob_enable", ndob_enable, ndob_enable);
    private_node.param("nmpc_ndob_bandwidth_x", ndob_bandwidth_x, ndob_bandwidth_x);
    private_node.param("nmpc_ndob_bandwidth_y", ndob_bandwidth_y, ndob_bandwidth_y);
    private_node.param("nmpc_ndob_bandwidth_z", ndob_bandwidth_z, ndob_bandwidth_z);
    private_node.param("nmpc_ndob_limit_x", ndob_limit_x, ndob_limit_x);
    private_node.param("nmpc_ndob_limit_y", ndob_limit_y, ndob_limit_y);
    private_node.param("nmpc_ndob_limit_z", ndob_limit_z, ndob_limit_z);
    if (!std::isfinite(roll_duration_sec) || roll_duration_sec < 0.8 ||
        !std::isfinite(roll_rate_limit) || roll_rate_limit <= 0.0) {
        ROS_ERROR("Invalid 3D roll parameters: duration=%.3f, rate_limit=%.3f",
                  roll_duration_sec, roll_rate_limit);
        return 1;
    }
    Eigen::Vector3f qpos(qpx, qpy, qpz), qvel(qvx, qvy, qvz);
    Eigen::Vector3f qquat(qqx, qqy, qqz), rw(rwx, rwy, rwz);
    NMPC_Ctrller_simple nmpc(0.02, {{-15.0, 15.0}}, {{-3.14, 3.14}}, 8, 0.05,
        10, 4, qpos, qvel, qquat, rw, rthrust, hover_thrust);
    std::unique_ptr<NMPC_Ctrller_simple> roll_nmpc;
    if (enable_cmd8)
    {
        const Eigen::Vector3f roll_qpos = qpos * 0.25f;
        const Eigen::Vector3f roll_qvel = qvel * 0.5f;
        const Eigen::Vector3f roll_qquat = qquat * 50.0f;
        const Eigen::Vector3f roll_rw = rw * 0.3f;
        roll_nmpc.reset(new NMPC_Ctrller_simple(
            0.02, {{-15.0, 15.0}}, {{-roll_rate_limit, roll_rate_limit}},
            8, 0.05, 10, 4, roll_qpos, roll_qvel, roll_qquat,
            roll_rw, rthrust, hover_thrust));
        roll_nmpc->configureDisturbanceObserver(false, Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(), nmpc_gravity);
    }
    nmpc.configureDisturbanceObserver(
        ndob_enable,
        Eigen::Vector3d(ndob_bandwidth_x, ndob_bandwidth_y, ndob_bandwidth_z),
        Eigen::Vector3d(ndob_limit_x, ndob_limit_y, ndob_limit_z),
        nmpc_gravity);
    ROS_INFO("NMPC NDOB %s: bandwidth=[%.2f %.2f %.2f] rad/s, limits=[%.2f %.2f %.2f] m/s^2",
             ndob_enable ? "enabled" : "disabled",
             ndob_bandwidth_x, ndob_bandwidth_y, ndob_bandwidth_z,
             ndob_limit_x, ndob_limit_y, ndob_limit_z);

    bool inverted_mode = false;
    ros::Time ndob_last_velocity_stamp;
    const auto reset_ndob = [&]() {
        nmpc.resetDisturbanceObserver(
            local_velocity, inverted_mode ? -nmpc_gravity : nmpc_gravity);
        ndob_last_velocity_stamp = local_velocity_stamp;
    };
    const auto update_ndob = [&]() {
        if (local_velocity_stamp.isZero())
        {
            return;
        }
        if (ndob_last_velocity_stamp.isZero())
        {
            nmpc.resetDisturbanceObserver(
                local_velocity, inverted_mode ? -nmpc_gravity : nmpc_gravity);
            ndob_last_velocity_stamp = local_velocity_stamp;
            return;
        }
        if (local_velocity_stamp <= ndob_last_velocity_stamp)
        {
            return;
        }
        nmpc.updateDisturbanceObserver(
            local_velocity, local_attitude,
            (local_velocity_stamp - ndob_last_velocity_stamp).toSec());
        ndob_last_velocity_stamp = local_velocity_stamp;
    };
    const auto publish_nmpc_state = [&](const std::vector<double> &desired,
                                        const NMPC_Ctrller_simple &controller) {
        fsm_ctrl::nmpc_state message;
        constexpr int kPredictionNodes = 9;
        constexpr int kStateSize = 10;

        for (int i = 0; i < kPredictionNodes; ++i)
        {
            const int offset = i * kStateSize;
            message.pos_ref[i].x = desired[offset];
            message.pos_ref[i].y = desired[offset + 1];
            message.pos_ref[i].z = desired[offset + 2];
            message.vel_ref[i].x = desired[offset + 3];
            message.vel_ref[i].y = desired[offset + 4];
            message.vel_ref[i].z = desired[offset + 5];
        }

        message.pos_fdb.x = local_position.x();
        message.pos_fdb.y = local_position.y();
        message.pos_fdb.z = local_position.z();
        message.vel_fdb.x = local_velocity.x();
        message.vel_fdb.y = local_velocity.y();
        message.vel_fdb.z = local_velocity.z();
        message.attitude_fdb.w = local_attitude.w();
        message.attitude_fdb.x = local_attitude.x();
        message.attitude_fdb.y = local_attitude.y();
        message.attitude_fdb.z = local_attitude.z();
        message.attitude_ref.w = desired[6];
        message.attitude_ref.x = desired[7];
        message.attitude_ref.y = desired[8];
        message.attitude_ref.z = desired[9];
        message.target = attitude_setpoint;
        const Eigen::Vector3d disturbance = controller.getDisturbanceEstimate();
        message.disturbance_accel.x = disturbance.x();
        message.disturbance_accel.y = disturbance.y();
        message.disturbance_accel.z = disturbance.z();
        message.ndob_active = controller.isDisturbanceObserverEnabled();
        nmpc_state_publisher.publish(message);
    };
    const auto publish_signed_command = [&](const Eigen::Vector3d &rates, double thrust) {
        const ros::Time stamp = ros::Time::now();
        geometry_msgs::TwistStamped rate_message;
        rate_message.header.stamp = stamp;
        // setpoint_attitude transforms ENU->NED; setpoint_raw transforms
        // base_link->aircraft. This permutation makes both send the same
        // aircraft-frame body rates to PX4.
        rate_message.twist.angular.x = -rates.y();
        rate_message.twist.angular.y = rates.x();
        rate_message.twist.angular.z = rates.z();
        mavros_msgs::Thrust thrust_message;
        thrust_message.header.stamp = stamp;
        thrust_message.thrust = std::max(-1.0, std::min(1.0, thrust));
        reverse_rate_publisher.publish(rate_message);
        reverse_thrust_publisher.publish(thrust_message);
        attitude_setpoint.header.stamp = stamp;
        attitude_setpoint.header.frame_id = "FCU";
        attitude_setpoint.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        attitude_setpoint.body_rate.x = rates.x();
        attitude_setpoint.body_rate.y = rates.y();
        attitude_setpoint.body_rate.z = rates.z();
        attitude_setpoint.thrust = thrust_message.thrust;
    };
    const auto signed_transport_ready = [&]() {
        bool reverse_thrust_enabled = false;
        bool use_quaternion = false;
        node.getParamCached("/mavros/setpoint_attitude/reverse_thrust",
                            reverse_thrust_enabled);
        node.getParamCached("/mavros/setpoint_attitude/use_quaternion",
                            use_quaternion);
        return reverse_thrust_enabled && !use_quaternion &&
            reverse_rate_publisher.getNumSubscribers() > 0 &&
            reverse_thrust_publisher.getNumSubscribers() > 0;
    };
    const auto publish_safe_attitude = [&]() {
        const double body_z_world_z =
            (local_attitude * Eigen::Vector3d::UnitZ()).z();
        publish_signed_command(Eigen::Vector3d::Zero(),
            body_z_world_z < 0.0 ? -hover_thrust : hover_thrust);
    };
    const auto run_nmpc = [&](const std::vector<double> &desired,
                              bool roll_mode = false) {
        NMPC_Ctrller_simple &controller = roll_mode ? *roll_nmpc : nmpc;
        const std::size_t expected_desired_size =
            static_cast<std::size_t>((controller.getNLPPredictStep() + 1) * controller.getNLPStateNum() +
                                     controller.getNLPPredictStep() * controller.getNLPInputNum());
        if (desired.size() != expected_desired_size)
        {
            ROS_ERROR_THROTTLE(1.0, "NMPC reference has %zu values, expected %zu",
                               desired.size(), expected_desired_size);
            publish_safe_attitude();
            return false;
        }
        if (!has_local_pose || !has_local_velocity)
        {
            ROS_WARN_THROTTLE(1.0, "NMPC is waiting for PX4 local pose and velocity");
            publish_safe_attitude();
            return false;
        }

        const std::vector<double> current{
            local_position.x(), local_position.y(), local_position.z(),
            local_velocity.x(), local_velocity.y(), local_velocity.z(),
            local_attitude.w(), local_attitude.x(), local_attitude.y(), local_attitude.z()};
        if (!roll_mode)
        {
            update_ndob();
        }
        try
        {
            controller.optimal_solution(current, desired);
        }
        catch (const std::exception &error)
        {
            ROS_ERROR_THROTTLE(1.0, "NMPC solve failed: %s", error.what());
            publish_safe_attitude();
            return false;
        }

        const Eigen::Vector3d rates = controller.getwCommand();
        const double thrust = controller.getAcc_zCommand();
        if (!rates.allFinite() || !std::isfinite(thrust))
        {
            ROS_ERROR_THROTTLE(1.0, "NMPC returned a non-finite command");
            publish_safe_attitude();
            return false;
        }

        publish_signed_command(rates, thrust);
        publish_nmpc_state(desired, controller);
        const Eigen::Vector3d disturbance = controller.getDisturbanceEstimate();
        ROS_INFO_THROTTLE(1.0,
                          "NMPC NDOB disturbance [m/s^2]: x=%+.3f y=%+.3f z=%+.3f",
                          disturbance.x(), disturbance.y(), disturbance.z());

        quadrotor_msgs::Px4ctrlDebug debug;
        debug.header.stamp = attitude_setpoint.header.stamp;
        debug.des_p_x = desired[0];
        debug.des_p_y = desired[1];
        debug.des_p_z = desired[2];
        debug.des_v_x = desired[3];
        debug.des_v_y = desired[4];
        debug.des_v_z = desired[5];
        debug.des_q_w = desired[6];
        debug.des_q_x = desired[7];
        debug.des_q_y = desired[8];
        debug.des_q_z = desired[9];
        Eigen::Quaterniond desired_quaternion(
            desired[6], desired[7], desired[8], desired[9]);
        desired_quaternion.normalize();
        const Eigen::Vector3d desired_acceleration =
            desired_quaternion * Eigen::Vector3d(0.0, 0.0, desired[93]) -
            Eigen::Vector3d(0.0, 0.0, nmpc_gravity);
        debug.des_a_x = desired_acceleration.x();
        debug.des_a_y = desired_acceleration.y();
        debug.des_a_z = desired_acceleration.z();
        debug.ideal_q_w = desired[6];
        debug.ideal_q_x = desired[7];
        debug.ideal_q_y = desired[8];
        debug.ideal_q_z = desired[9];
        debug.fb_a_x = latest_imu.linear_acceleration.x;
        debug.fb_a_y = latest_imu.linear_acceleration.y;
        debug.fb_a_z = latest_imu.linear_acceleration.z;
        debug.fb_rate_x = latest_imu.angular_velocity.x;
        debug.fb_rate_y = latest_imu.angular_velocity.y;
        debug.fb_rate_z = latest_imu.angular_velocity.z;
        debug.ideal_rate_x = rates.x();
        debug.ideal_rate_y = rates.y();
        debug.ideal_rate_z = rates.z();
        debug.des_thr = attitude_setpoint.thrust;
        debug.ideal_thr = desired[93] * hover_thrust / nmpc_gravity;
        debug.hover_percentage = hover_thrust;
        debug.thr2acc = nmpc_gravity / std::max(1e-6, hover_thrust);
        debug.voltage = latest_battery.voltage;

        Eigen::Quaterniond attitude_error =
            desired_quaternion * local_attitude.conjugate();
        if (attitude_error.w() < 0.0)
        {
            attitude_error.coeffs() *= -1.0;
        }
        const Eigen::AngleAxisd execution_error(attitude_error);
        debug.exec_err_axisang_x = execution_error.axis().x();
        debug.exec_err_axisang_y = execution_error.axis().y();
        debug.exec_err_axisang_z = execution_error.axis().z();
        debug.exec_err_axisang_ang = execution_error.angle();
        const Eigen::AngleAxisd feedback_attitude(local_attitude);
        debug.fb_axisang_x = feedback_attitude.axis().x();
        debug.fb_axisang_y = feedback_attitude.axis().y();
        debug.fb_axisang_z = feedback_attitude.axis().z();
        debug.fb_axisang_ang = feedback_attitude.angle();
        control_debug_publisher.publish(debug);
        return true;
    };
    // 与 Jiangyin/nmpc 一致的有界低频 XY 位置修正。修正量单位为米，
    // 对整个 NMPC 参考窗口统一平移，不改变轨迹形状、速度和碰撞时序。
    Eigen::Vector2d xy_integral_correction = Eigen::Vector2d::Zero();
    bool initial_orientation_known = false;
    bool roll_active = false;
    bool roll_started_inverted = false;
    double ground_reference_z = 0.0;
    bool ground_reference_valid = false;
    double inverted_figure_height = hover_z;
    Eigen::Vector3d inverted_land_entry = Eigen::Vector3d::Zero();
    double inverted_land_yaw = 0.0;
    ros::Time inverted_land_start;
    ros::Time inverted_ground_since;
    ros::Time inverted_last_disarm_request;
    ros::Time xy_integral_last_update;
    const auto reset_xy_integral = [&]() {
        xy_integral_correction.setZero();
        xy_integral_last_update = ros::Time();
    };
    const auto update_xy_integral = [&](const Eigen::Vector2d &position_reference) {
        const ros::Time now = ros::Time::now();
        if (xy_integral_last_update.isZero())
        {
            xy_integral_last_update = now;
            return;
        }

        const double dt = (now - xy_integral_last_update).toSec();
        xy_integral_last_update = now;
        if (dt <= 0.0 || dt > 0.10)
        {
            return;
        }

        const Eigen::Vector2d position_error =
            position_reference - local_position.head<2>();
        if (enable_xy_integral && position_error.norm() <= xy_integral_max_error)
        {
            xy_integral_correction += xy_integral_gain * position_error * dt;
        }
        xy_integral_correction *= std::max(0.0, 1.0 - xy_integral_leak * dt);
        xy_integral_correction.x() = std::max(
            -xy_integral_limit, std::min(xy_integral_limit, xy_integral_correction.x()));
        xy_integral_correction.y() = std::max(
            -xy_integral_limit, std::min(xy_integral_limit, xy_integral_correction.y()));
    };
    const auto orient_for_mode = [&](const Eigen::Quaterniond &upright) {
        if (!inverted_mode)
        {
            return upright;
        }
        // R_inverted = R_upright * Rx(pi). With signed thrust -T,
        // R_inverted * (0, 0, -T) equals R_upright * (0, 0, T).
        // World-frame planner position/velocity/acceleration stay unchanged.
        return Eigen::Quaterniond(upright * Eigen::Quaterniond(
            Eigen::AngleAxisd(3.14159265358979323846,
                              Eigen::Vector3d::UnitX())));
    };
    const auto make_hold_reference = [&](const Eigen::Vector3d &position, double yaw) {
        std::vector<double> desired;
        const double half_yaw = 0.5 * yaw;
        const Eigen::Quaterniond upright_q(
            std::cos(half_yaw), 0.0, 0.0, std::sin(half_yaw));
        const Eigen::Quaterniond desired_q = inverted_mode
            ? upright_q * Eigen::Quaterniond(
                Eigen::AngleAxisd(3.14159265358979323846, Eigen::Vector3d::UnitX()))
            : upright_q;
        for (int i = 0; i < 9; ++i)
        {
            desired.insert(desired.end(), {
                position.x(), position.y(), position.z(),
                0.0, 0.0, 0.0,
                desired_q.w(), desired_q.x(), desired_q.y(), desired_q.z()});
        }
        for (int i = 0; i < 8; ++i)
        {
            desired.insert(desired.end(), {
                0.0, 0.0, 0.0,
                inverted_mode ? -nmpc_gravity : nmpc_gravity});
        }
        return desired;
    };
    const auto nmpc_hover = [&]() {
        update_xy_integral(Eigen::Vector2d(hover_x, hover_y));
        run_nmpc(make_hold_reference(
            Eigen::Vector3d(
                hover_x + xy_integral_correction.x(),
                hover_y + xy_integral_correction.y(),
                hover_z),
            hover_yaw));
    };
    // A body-X roll with nonzero thrust necessarily accelerates sideways.
    // Build one dynamically consistent position/velocity/attitude reference.
    struct RollPoint {
        double lateral = 0.0, height = 0.0;
        double lateral_speed = 0.0, vertical_speed = 0.0;
    };
    const int roll_segments = std::max(2, static_cast<int>(
        std::ceil(std::max(0.01, roll_duration_sec) / 0.01)));
    const double roll_dt = roll_duration_sec / roll_segments;
    const auto roll_angle = [](double s) {
        return 3.14159265358979323846 *
            (10.0*s*s*s - 15.0*s*s*s*s + 6.0*s*s*s*s*s);
    };
    const auto roll_rate = [&](double s) {
        return 3.14159265358979323846 *
            (30.0*s*s - 60.0*s*s*s + 30.0*s*s*s*s) / roll_duration_sec;
    };
    const auto roll_thrust = [&](double s, double gain) {
        const double c = std::cos(roll_angle(s));
        const double shape = std::sin(3.14159265358979323846 * s);
        const double raw = nmpc_gravity * c *
            (1.2 + gain * shape * shape) / (c*c + 0.2);
        return std::max(-15.0, std::min(15.0, raw));
    };
    const auto roll_accel = [&](double s, double gain) {
        const double angle = roll_angle(s);
        const double thrust = roll_thrust(s, gain);
        return Eigen::Vector2d(-std::sin(angle) * thrust,
                               std::cos(angle) * thrust - nmpc_gravity);
    };
    const auto final_vertical_speed = [&](double gain) {
        double result = 0.0;
        for (int i = 0; i < roll_segments; ++i) {
            result += 0.5 * roll_dt *
                (roll_accel(static_cast<double>(i) / roll_segments, gain).y() +
                 roll_accel(static_cast<double>(i + 1) / roll_segments, gain).y());
        }
        return result;
    };
    double roll_gain_low = 0.0, roll_gain_high = 5.0;
    for (int i = 0; i < 40; ++i) {
        const double middle = 0.5 * (roll_gain_low + roll_gain_high);
        if (final_vertical_speed(middle) < 0.0) roll_gain_low = middle;
        else roll_gain_high = middle;
    }
    const double roll_gain = 0.5 * (roll_gain_low + roll_gain_high);
    std::vector<RollPoint> roll_profile(roll_segments + 1);
    for (int i = 0; i < roll_segments; ++i) {
        const Eigen::Vector2d a0 = roll_accel(
            static_cast<double>(i) / roll_segments, roll_gain);
        const Eigen::Vector2d a1 = roll_accel(
            static_cast<double>(i + 1) / roll_segments, roll_gain);
        const Eigen::Vector2d average = 0.5 * (a0 + a1);
        const RollPoint &before = roll_profile[i];
        RollPoint &after = roll_profile[i + 1];
        after.lateral_speed = before.lateral_speed + average.x() * roll_dt;
        after.vertical_speed = before.vertical_speed + average.y() * roll_dt;
        after.lateral = before.lateral +
            0.5 * (before.lateral_speed + after.lateral_speed) * roll_dt;
        after.height = before.height +
            0.5 * (before.vertical_speed + after.vertical_speed) * roll_dt;
    }
    double roll_peak_rise = 0.0, roll_min_height_offset = 0.0;
    for (const RollPoint &point : roll_profile) {
        roll_peak_rise = std::max(roll_peak_rise, point.height);
        roll_min_height_offset = std::min(roll_min_height_offset, point.height);
    }
    ROS_INFO("cmd 8 roll reference: duration=%.2f s, lateral shift=%.2f m, "
             "height range=[%.2f, %.2f] m; verify free space around aircraft",
             roll_duration_sec, roll_profile.back().lateral,
             roll_min_height_offset, roll_peak_rise);
    const auto make_roll_reference = [&](const Eigen::Vector3d &position,
                                         double yaw, double elapsed,
                                         bool started_inverted) {
        std::vector<double> desired;
        const Eigen::Quaterniond yaw_rotation(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        const Eigen::Vector3d lateral_axis =
            yaw_rotation * Eigen::Vector3d::UnitY();
        const auto sample = [&](double time) {
            const double s = std::max(0.0, std::min(1.0, time / roll_duration_sec));
            const double index = s * roll_segments;
            const int lower = std::min(roll_segments - 1, static_cast<int>(index));
            const double alpha = index - lower;
            const RollPoint &a = roll_profile[lower], &b = roll_profile[lower + 1];
            RollPoint point;
            point.lateral = a.lateral + alpha * (b.lateral - a.lateral);
            point.height = a.height + alpha * (b.height - a.height);
            point.lateral_speed = a.lateral_speed +
                alpha * (b.lateral_speed - a.lateral_speed);
            point.vertical_speed = a.vertical_speed +
                alpha * (b.vertical_speed - a.vertical_speed);
            return point;
        };
        for (int i = 0; i <= nmpc.getNLPPredictStep(); ++i) {
            const double time = elapsed + i * nmpc.getNLPOnestepTime();
            const double s = std::max(0.0, std::min(1.0, time / roll_duration_sec));
            const RollPoint point = sample(time);
            const double lateral_sign = started_inverted ? -1.0 : 1.0;
            const Eigen::Vector3d target = position +
                lateral_axis * (lateral_sign * point.lateral) +
                Eigen::Vector3d(0.0, 0.0, point.height);
            const Eigen::Vector3d velocity =
                lateral_axis * (lateral_sign * point.lateral_speed) +
                Eigen::Vector3d(0.0, 0.0, point.vertical_speed);
            const Eigen::Quaterniond q = yaw_rotation * Eigen::Quaterniond(
                Eigen::AngleAxisd(started_inverted
                    ? 3.14159265358979323846 - roll_angle(s) : roll_angle(s),
                    Eigen::Vector3d::UnitX()));
            desired.insert(desired.end(), {
                target.x(), target.y(), target.z(),
                velocity.x(), velocity.y(), velocity.z(),
                q.w(), q.x(), q.y(), q.z()});
        }
        for (int i = 0; i < nmpc.getNLPPredictStep(); ++i) {
            const double time = elapsed + i * nmpc.getNLPOnestepTime();
            const double s = std::max(0.0, std::min(1.0, time / roll_duration_sec));
            desired.insert(desired.end(), {
                started_inverted ? -roll_rate(s) : roll_rate(s), 0.0, 0.0,
                started_inverted ? -roll_thrust(s, roll_gain)
                                 : roll_thrust(s, roll_gain)});
        }
        return desired;
    };
    const auto make_inverted_land_reference = [&](double elapsed) {
        constexpr double kPi = 3.14159265358979323846;
        const Eigen::Quaterniond attitude =
            Eigen::Quaterniond(Eigen::AngleAxisd(
                inverted_land_yaw, Eigen::Vector3d::UnitZ())) *
            Eigen::Quaterniond(Eigen::AngleAxisd(
                kPi, Eigen::Vector3d::UnitX()));
        std::vector<double> desired;
        for (int i = 0; i <= nmpc.getNLPPredictStep(); ++i)
        {
            const double time = elapsed + i * nmpc.getNLPOnestepTime();
            const double floor_z = ground_reference_z - 0.05;
            const double z = std::max(floor_z,
                inverted_land_entry.z() - inverted_land_speed * time);
            const double vz = z > floor_z ? -inverted_land_speed : 0.0;
            desired.insert(desired.end(), {
                inverted_land_entry.x(), inverted_land_entry.y(), z,
                0.0, 0.0, vz,
                attitude.w(), attitude.x(), attitude.y(), attitude.z()});
        }
        for (int i = 0; i < nmpc.getNLPPredictStep(); ++i)
        {
            desired.insert(desired.end(), {0.0, 0.0, 0.0, -nmpc_gravity});
        }
        return desired;
    };
    const auto nmpc_planner_track = [&](const Eigen::Vector3d &fallback_hold_position) {
        const bool horizon_is_fresh = has_planner_horizon &&
            planner_horizon.points.size() >= static_cast<size_t>(nmpc.getNLPPredictStep() + 1) &&
            (ros::Time::now() - planner_horizon_stamp).toSec() <= planner_command_timeout;
        const bool command_is_fresh = has_planner_command &&
            (ros::Time::now() - planner_command_stamp).toSec() <= planner_command_timeout;
        if (!horizon_is_fresh && !command_is_fresh)
        {
            ROS_WARN_THROTTLE(1.0, "Planner horizon and command are missing or stale; NMPC holds the entry position");
            run_nmpc(make_hold_reference(
                Eigen::Vector3d(
                    fallback_hold_position.x() + xy_integral_correction.x(),
                    fallback_hold_position.y() + xy_integral_correction.y(),
                    fallback_hold_position.z()),
                0.0));
            return;
        }

        std::vector<double> desired;
        if (horizon_is_fresh)
        {
            const quadrotor_msgs::PositionCommand &first = planner_horizon.points.front();
            update_xy_integral(Eigen::Vector2d(first.position.x, first.position.y));

            for (int i = 0; i <= nmpc.getNLPPredictStep(); ++i)
            {
                const quadrotor_msgs::PositionCommand &point =
                    planner_horizon.points[static_cast<size_t>(i)];
                Eigen::Vector3d thrust_acceleration(
                    point.acceleration.x,
                    point.acceleration.y,
                    point.acceleration.z + nmpc_gravity);
                if (thrust_acceleration.norm() < 1e-3)
                {
                    thrust_acceleration = Eigen::Vector3d(0.0, 0.0, nmpc_gravity);
                }
                Eigen::Quaterniond desired_attitude(
                    DiffFlat(thrust_acceleration, point.yaw));
                desired_attitude.normalize();
                desired_attitude = orient_for_mode(desired_attitude);
                desired.insert(desired.end(), {
                    point.position.x + xy_integral_correction.x(),
                    point.position.y + xy_integral_correction.y(),
                    point.position.z,
                    point.velocity.x, point.velocity.y, point.velocity.z,
                    desired_attitude.w(), desired_attitude.x(),
                    desired_attitude.y(), desired_attitude.z()});
            }
            for (int i = 0; i < nmpc.getNLPPredictStep(); ++i)
            {
                const quadrotor_msgs::PositionCommand &point =
                    planner_horizon.points[static_cast<size_t>(i)];
                Eigen::Vector3d thrust_acceleration(
                    point.acceleration.x,
                    point.acceleration.y,
                    point.acceleration.z + nmpc_gravity);
                if (thrust_acceleration.norm() < 1e-3)
                {
                    thrust_acceleration = Eigen::Vector3d(0.0, 0.0, nmpc_gravity);
                }
                desired.insert(desired.end(), {
                    0.0, 0.0,
                    inverted_mode ? -point.yaw_dot : point.yaw_dot,
                    inverted_mode ? -thrust_acceleration.norm() : thrust_acceleration.norm()});
            }
            run_nmpc(desired);
            return;
        }

        const Eigen::Vector3d p0(
            planner_command.position.x,
            planner_command.position.y,
            planner_command.position.z);
        update_xy_integral(p0.head<2>());
        const Eigen::Vector3d v0(
            planner_command.velocity.x,
            planner_command.velocity.y,
            planner_command.velocity.z);
        const Eigen::Vector3d acceleration(
            planner_command.acceleration.x,
            planner_command.acceleration.y,
            planner_command.acceleration.z);
        Eigen::Vector3d thrust_acceleration =
            acceleration + Eigen::Vector3d(0.0, 0.0, nmpc_gravity);
        if (thrust_acceleration.norm() < 1e-3)
        {
            thrust_acceleration = Eigen::Vector3d(0.0, 0.0, nmpc_gravity);
        }
        const double collective_acceleration = thrust_acceleration.norm();
        Eigen::Quaterniond desired_attitude(
            DiffFlat(thrust_acceleration, planner_command.yaw));
        desired_attitude.normalize();
        desired_attitude = orient_for_mode(desired_attitude);

        for (int i = 0; i < 9; ++i)
        {
            const double prediction_time = i * nmpc.getNLPOnestepTime();
            const Eigen::Vector3d position =
                p0 + v0 * prediction_time + 0.5 * acceleration * prediction_time * prediction_time;
            const Eigen::Vector3d velocity = v0 + acceleration * prediction_time;
            desired.insert(desired.end(), {
                position.x() + xy_integral_correction.x(),
                position.y() + xy_integral_correction.y(),
                position.z(),
                velocity.x(), velocity.y(), velocity.z(),
                desired_attitude.w(), desired_attitude.x(),
                desired_attitude.y(), desired_attitude.z()});
        }
        for (int i = 0; i < 8; ++i)
        {
            desired.insert(desired.end(), {
                0.0, 0.0,
                inverted_mode ? -planner_command.yaw_dot : planner_command.yaw_dot,
                inverted_mode ? -collective_acceleration : collective_acceleration});
        }
        run_nmpc(desired);
    };
    const auto nmpc_figure_eight = [&](int step) {
        const double current_phase = step * 0.02;
        update_xy_integral(Eigen::Vector2d(
            0.75 * std::sin(current_phase),
            0.375 * std::sin(2.0 * current_phase)));
        std::vector<double> desired;
        const Eigen::Quaterniond figure_eight_attitude = orient_for_mode(
            Eigen::Quaterniond::Identity());
        for (int i = 0; i < 9; ++i)
        {
            const double phase = step * 0.02 + i * nmpc.getNLPOnestepTime();
            desired.insert(desired.end(), {
                0.75 * std::sin(phase) + xy_integral_correction.x(),
                0.375 * std::sin(2.0 * phase) + xy_integral_correction.y(),
                inverted_figure_height,
                0.75 * std::cos(phase), 0.75 * std::cos(2.0 * phase), 0.0,
                figure_eight_attitude.w(), figure_eight_attitude.x(),
                figure_eight_attitude.y(), figure_eight_attitude.z()});
        }
        for (int i = 0; i < 8; ++i)
        {
            desired.insert(desired.end(), {
                0.0, 0.0, 0.0,
                inverted_mode ? -nmpc_gravity : nmpc_gravity});
        }
        run_nmpc(desired);
    };

    std::thread(ListenForUdpCommands, kUdpPort).detach();

    ros::Rate rate(kControlRateHz);
    for (int i = 0; ros::ok() && i < 100; ++i)
    {
        // Prime OFFBOARD with the same signed attitude transport used in flight.
        // A position setpoint here could select an upright-only PX4 controller.
        publish_signed_command(Eigen::Vector3d::Zero(), 0.0);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();
    int trajectory_step = 0;
    int previous_command = 0;
    Eigen::Vector3d planner_fallback_hold = local_position;
    Eigen::Vector3d roll_hold_position = Eigen::Vector3d::Zero();
    double roll_hold_yaw = 0.0;
    ros::Time roll_start_time;

    while (ros::ok())
    {
        ros::spinOnce();
        if (!initial_orientation_known || (!current_state.armed && !roll_active))
        {
            const ros::Time now = ros::Time::now();
            const double body_z_world_z =
                (local_attitude * Eigen::Vector3d::UnitZ()).z();
            if (has_local_pose &&
                std::abs((now - local_pose_stamp).toSec()) < roll_state_timeout_sec &&
                local_position.allFinite() &&
                std::isfinite(body_z_world_z) &&
                std::abs(body_z_world_z) > 0.95)
            {
                const bool detected_inverted = body_z_world_z < 0.0;
                if (!initial_orientation_known || inverted_mode != detected_inverted)
                {
                    ROS_WARN("3D ground orientation detected: %s; cmd 3 uses %s thrust",
                             detected_inverted ? "inverted" : "upright",
                             detected_inverted ? "reverse" : "forward");
                    inverted_mode = detected_inverted;
                    nmpc.resetWarmStart(
                        inverted_mode ? -nmpc_gravity : nmpc_gravity);
                    reset_ndob();
                }
                initial_orientation_known = true;
                ground_reference_z = local_position.z();
                ground_reference_valid = true;
            }
            else if (!initial_orientation_known ||
                     command.load(std::memory_order_relaxed) != 0)
            {
                ROS_ERROR_THROTTLE(1.0,
                    "Waiting for fresh, nearly level upright/inverted pose before accepting flight commands");
                publish_signed_command(Eigen::Vector3d::Zero(), 0.0);
                rate.sleep();
                continue;
            }
        }
        int active_command = command.load(std::memory_order_relaxed);
        if (roll_active && active_command != 8)
        {
            ROS_WARN_THROTTLE(1.0,
                "cmd 8 is still rolling; waiting for stable target attitude");
            command.store(8, std::memory_order_relaxed);
            active_command = 8;
        }
        if (active_command == 8 && previous_command != 8)
        {
            const ros::Time now = ros::Time::now();
            const bool state_fresh = has_local_pose && has_local_velocity &&
                (now - local_pose_stamp).toSec() < roll_state_timeout_sec &&
                (now - local_velocity_stamp).toSec() < roll_state_timeout_sec;
            const Eigen::Vector3d body_z =
                local_attitude * Eigen::Vector3d::UnitZ();
            const bool ready = enable_cmd8 && previous_command == 3 &&
                current_state.armed && current_state.mode == "OFFBOARD" &&
                signed_transport_ready() &&
                roll_duration_sec >= 0.8 &&
                roll_rate_limit >= 1.875 * 3.14159265358979323846 /
                                       roll_duration_sec &&
                roll_min_height > 0.0 &&
                final_vertical_speed(roll_gain_high) >= -0.01 &&
                state_fresh && local_position.allFinite() &&
                local_velocity.allFinite() &&
                (inverted_mode ? body_z.z() < -0.95 : body_z.z() > 0.95) &&
                local_position.z() + roll_min_height_offset >= roll_min_height &&
                local_velocity.norm() <= roll_max_entry_speed;
            if (!ready)
            {
                ROS_ERROR("cmd 8 rejected: check stable cmd 3 hover, reverse-thrust MAVROS, roll time/rate, and height");
                command.store(previous_command, std::memory_order_relaxed);
                active_command = previous_command;
            }
            else
            {
                roll_hold_position = local_position;
                roll_started_inverted = inverted_mode;
                const Eigen::Quaterniond upright_equivalent = inverted_mode
                    ? Eigen::Quaterniond(local_attitude * Eigen::Quaterniond(
                        Eigen::AngleAxisd(3.14159265358979323846,
                                          Eigen::Vector3d::UnitX())))
                    : local_attitude;
                roll_hold_yaw = std::atan2(
                    2.0 * (upright_equivalent.w() * upright_equivalent.z() +
                           upright_equivalent.x() * upright_equivalent.y()),
                    1.0 - 2.0 * (upright_equivalent.y() * upright_equivalent.y() +
                                 upright_equivalent.z() * upright_equivalent.z()));
                roll_start_time = now;
                roll_active = true;
                roll_nmpc->resetWarmStart(
                    roll_started_inverted ? -nmpc_gravity : nmpc_gravity);
                roll_nmpc->resetDisturbanceObserver(
                    local_velocity,
                    roll_started_inverted ? -nmpc_gravity : nmpc_gravity);
                ROS_WARN("cmd 8 accepted: 180-degree roll %s -> %s, then cmd 3 hover",
                         roll_started_inverted ? "inverted" : "upright",
                         roll_started_inverted ? "upright" : "inverted");
            }
        }
        if (active_command == 4 && previous_command != 4 && inverted_mode)
        {
            inverted_land_entry = local_position;
            const Eigen::Quaterniond upright_equivalent = local_attitude *
                Eigen::Quaterniond(Eigen::AngleAxisd(
                    3.14159265358979323846, Eigen::Vector3d::UnitX()));
            inverted_land_yaw = std::atan2(
                2.0 * (upright_equivalent.w() * upright_equivalent.z() +
                       upright_equivalent.x() * upright_equivalent.y()),
                1.0 - 2.0 * (upright_equivalent.y() * upright_equivalent.y() +
                             upright_equivalent.z() * upright_equivalent.z()));
            inverted_land_start = ros::Time::now();
            inverted_ground_since = ros::Time();
            inverted_last_disarm_request = ros::Time();
            is_landed = false;
            ROS_WARN("cmd 4: inverted NMPC descent started");
        }
        if (active_command != previous_command)
        {
            const bool was_nmpc_active =
                previous_command == 3 || previous_command == 5 ||
                previous_command == 6 || previous_command == 7 ||
                previous_command == 8;
            const bool is_nmpc_active =
                active_command == 3 || active_command == 5 ||
                active_command == 6 || active_command == 7 ||
                active_command == 8;
            if (!was_nmpc_active && is_nmpc_active)
            {
                reset_ndob();
            }
            else if (was_nmpc_active && !is_nmpc_active)
            {
                reset_ndob();
            }
            trajectory_step = 0;
            if (active_command == 7 && has_local_pose)
            {
                inverted_figure_height = local_position.z();
            }
            reset_xy_integral();
            const bool is_trajectory_command =
                active_command == 5 ||
                (active_command == 6 && mission_mode == "multi");
            const bool was_trajectory_command =
                previous_command == 5 ||
                (previous_command == 6 && mission_mode == "multi");
            if (active_command == 5 || active_command == 6)
            {
                planner_fallback_hold = local_position;
                // A reference from the previous tracking session must not be reused
                // while the bridge processes the new start trigger.
                has_planner_command = false;
                has_planner_horizon = false;
            }
            if (is_trajectory_command || was_trajectory_command)
            {
                geometry_msgs::PoseStamped trigger;
                trigger.header.stamp = ros::Time::now();
                trigger.header.frame_id = is_trajectory_command
                    ? (mission_mode == "multi" ? "segment" : "start")
                    : "stop";
                trigger.pose = position_setpoint.pose;
                trigger.pose.position.x = local_position.x();
                trigger.pose.position.y = local_position.y();
                trigger.pose.position.z = local_position.z();
                trigger.pose.orientation.w = local_attitude.w();
                trigger.pose.orientation.x = local_attitude.x();
                trigger.pose.orientation.y = local_attitude.y();
                trigger.pose.orientation.z = local_attitude.z();
                planner_start_trigger_publisher.publish(trigger);
            }
            ROS_INFO("FSM command changed: %d -> %d", previous_command, active_command);
            previous_command = active_command;
        }
        std_msgs::Int32 command_state;
        command_state.data = active_command;
        command_state_publisher.publish(command_state);

        if (active_command != 0 &&
            (active_command != 4 || inverted_mode) &&
            !signed_transport_ready())
        {
            ROS_ERROR_THROTTLE(1.0,
                "3D MAVROS attitude transport unavailable; refusing FSM command %d",
                active_command);
            rate.sleep();
            continue;
        }

        switch (active_command)
        {
        case 1:
            if (inverted_mode)
            {
                RequestOffboardAndArm(
                    set_mode_client, arming_client, offboard_mode,
                    arm_command, last_request);
                publish_signed_command(Eigen::Vector3d::Zero(), -0.02);
                break;
            }
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);

            publish_signed_command(Eigen::Vector3d::Zero(), 0.02);
            break;

        case 2:
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);
            if (inverted_mode)
            {
                run_nmpc(make_hold_reference(
                    Eigen::Vector3d(0.0, 0.0, 1.0), hover_yaw));
            }
            else
            {
                SetPosition(0.0, 0.0, 1.0);
                position_publisher.publish(position_setpoint);
            }
            break;

        case 3:
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);
            nmpc_hover();
            break;

        case 4:
            if (inverted_mode)
            {
                if (!current_state.armed)
                {
                    publish_signed_command(Eigen::Vector3d::Zero(), 0.0);
                    break;
                }
                run_nmpc(make_inverted_land_reference(
                    (ros::Time::now() - inverted_land_start).toSec()));
                const ros::Time landing_now = ros::Time::now();
                const bool on_ground = has_extended_state &&
                    (landing_now - extended_state_stamp).toSec() < 0.5 &&
                    (landing_now - local_pose_stamp).toSec() < roll_state_timeout_sec &&
                    (landing_now - local_velocity_stamp).toSec() < roll_state_timeout_sec &&
                    current_extended_state.landed_state ==
                        mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND &&
                    ground_reference_valid &&
                    std::abs(local_position.z() - ground_reference_z) < 0.25 &&
                    std::abs(local_velocity.z()) < 0.15;
                if (on_ground)
                {
                    if (inverted_ground_since.isZero())
                    {
                        inverted_ground_since = ros::Time::now();
                    }
                    else if ((ros::Time::now() - inverted_ground_since).toSec() > 0.5 &&
                             (inverted_last_disarm_request.isZero() ||
                              (ros::Time::now() - inverted_last_disarm_request).toSec() > 1.0))
                    {
                        inverted_last_disarm_request = ros::Time::now();
                        arm_command.request.value = false;
                        if (arming_client.call(arm_command) &&
                            arm_command.response.success)
                        {
                            ROS_WARN("Inverted landing complete; disarmed");
                        }
                    }
                }
                else
                {
                    inverted_ground_since = ros::Time();
                }
            }
            else if (!is_landed)
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
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);
            nmpc_planner_track(planner_fallback_hold);
            break;

        case 6:
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);
            if (mission_mode == "multi")
            {
                nmpc_planner_track(planner_fallback_hold);
            }
            else
            {
                ROS_WARN_THROTTLE(2.0, "cmd 6 requires Mission.mode=multi; NMPC holds entry position");
                nmpc_planner_track(planner_fallback_hold);
            }
            break;

        case 7:
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);
            nmpc_figure_eight(trajectory_step);
            {
                const double phase = 0.02 * trajectory_step;
                geometry_msgs::PoseStamped reference_pose;
                geometry_msgs::TwistStamped reference_velocity;
                reference_pose.header.stamp = reference_velocity.header.stamp = ros::Time::now();
                reference_pose.header.frame_id = reference_velocity.header.frame_id = "map";
                reference_pose.pose.position.x =
                    0.75 * std::sin(phase) + xy_integral_correction.x();
                reference_pose.pose.position.y =
                    0.375 * std::sin(2.0 * phase) + xy_integral_correction.y();
                reference_pose.pose.position.z = inverted_figure_height;
                reference_pose.pose.orientation.w = inverted_mode ? 0.0 : 1.0;
                reference_pose.pose.orientation.x = inverted_mode ? 1.0 : 0.0;
                reference_velocity.twist.linear.x = 0.75 * std::cos(phase);
                reference_velocity.twist.linear.y = 0.75 * std::cos(2.0 * phase);
                trajectory_reference_publisher.publish(reference_pose);
                trajectory_reference_velocity_publisher.publish(reference_velocity);
            }
            trajectory_step = (trajectory_step + 1) % 315;
            break;

        case 8:
            run_nmpc(make_roll_reference(
                roll_hold_position, roll_hold_yaw,
                (ros::Time::now() - roll_start_time).toSec(),
                roll_started_inverted), true);
            if (roll_active &&
                (ros::Time::now() - roll_start_time).toSec() >= roll_duration_sec &&
                (roll_started_inverted
                    ? (local_attitude * Eigen::Vector3d::UnitZ()).z() > 0.95
                    : (local_attitude * Eigen::Vector3d::UnitZ()).z() < -0.95) &&
                std::abs(latest_imu.angular_velocity.x) < 0.5 &&
                local_velocity.norm() < 0.5)
            {
                inverted_mode = !roll_started_inverted;
                roll_active = false;
                inverted_figure_height = roll_hold_position.z();
                reset_ndob();
                nmpc.resetWarmStart(
                    inverted_mode ? -nmpc_gravity : nmpc_gravity);
                command.store(3, std::memory_order_relaxed);
                ROS_WARN("cmd 8 complete; switching to cmd 3 %s hover",
                         inverted_mode ? "inverted" : "upright");
            }
            break;

        default:
            break;
        }

        rate.sleep();
    }

    return 0;
}
