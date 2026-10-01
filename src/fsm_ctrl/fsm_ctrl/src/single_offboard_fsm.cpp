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
sensor_msgs::Imu latest_imu;
sensor_msgs::BatteryState latest_battery;
Eigen::Vector3d local_position = Eigen::Vector3d::Zero();
Eigen::Vector3d local_velocity = Eigen::Vector3d::Zero();
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

void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &message)
{
    local_position = Eigen::Vector3d(
        message->pose.position.x,
        message->pose.position.y,
        message->pose.position.z);
    local_attitude = Eigen::Quaterniond(message->pose.orientation.w, message->pose.orientation.x, message->pose.orientation.y, message->pose.orientation.z);
    local_attitude.normalize();
    has_local_pose = true;
}

void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &message)
{
    local_velocity = Eigen::Vector3d(message->twist.linear.x, message->twist.linear.y, message->twist.linear.z);
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
    private_node.param("planner_command_topic", planner_command_topic, planner_command_topic);
    private_node.param("planner_horizon_topic", planner_horizon_topic, planner_horizon_topic);

    const ros::Publisher position_publisher =
        node.advertise<geometry_msgs::PoseStamped>(
            "/mavros/setpoint_position/local", 10);
    const ros::Publisher attitude_publisher =
        node.advertise<mavros_msgs::AttitudeTarget>(
            "/mavros/setpoint_raw/attitude", 10);
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
    double hover_x = 0.0, hover_y = 0.0, hover_z = 0.5, hover_yaw = 0.0;
    bool enable_xy_integral = true;
    double xy_integral_gain = 0.08;
    double xy_integral_limit = 0.10;
    double xy_integral_leak = 0.01;
    double xy_integral_max_error = 0.30;
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
    private_node.param("nmpc_enable_xy_integral", enable_xy_integral, enable_xy_integral);
    private_node.param("nmpc_xy_integral_gain", xy_integral_gain, xy_integral_gain);
    private_node.param("nmpc_xy_integral_limit", xy_integral_limit, xy_integral_limit);
    private_node.param("nmpc_xy_integral_leak", xy_integral_leak, xy_integral_leak);
    private_node.param("nmpc_xy_integral_max_error", xy_integral_max_error, xy_integral_max_error);
    Eigen::Vector3f qpos(qpx, qpy, qpz), qvel(qvx, qvy, qvz);
    Eigen::Vector3f qquat(qqx, qqy, qqz), rw(rwx, rwy, rwz);
    NMPC_Ctrller_simple nmpc(0.02, {{0.0, 15.0}}, {{-3.14, 3.14}}, 8, 0.05,
        10, 4, qpos, qvel, qquat, rw, rthrust, hover_thrust);
    const auto publish_nmpc_state = [&](const std::vector<double> &desired) {
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
        nmpc_state_publisher.publish(message);
    };
    const auto publish_safe_attitude = [&]() {
        attitude_setpoint.header.stamp = ros::Time::now();
        attitude_setpoint.header.frame_id = "FCU";
        attitude_setpoint.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        attitude_setpoint.body_rate.x = 0.0;
        attitude_setpoint.body_rate.y = 0.0;
        attitude_setpoint.body_rate.z = 0.0;
        attitude_setpoint.thrust = std::max(0.0, std::min(1.0, hover_thrust));
        attitude_publisher.publish(attitude_setpoint);
    };
    const auto run_nmpc = [&](const std::vector<double> &desired) {
        const std::size_t expected_desired_size =
            static_cast<std::size_t>((nmpc.getNLPPredictStep() + 1) * nmpc.getNLPStateNum() +
                                     nmpc.getNLPPredictStep() * nmpc.getNLPInputNum());
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
        try
        {
            nmpc.optimal_solution(current, desired);
        }
        catch (const std::exception &error)
        {
            ROS_ERROR_THROTTLE(1.0, "NMPC solve failed: %s", error.what());
            publish_safe_attitude();
            return false;
        }

        const Eigen::Vector3d rates = nmpc.getwCommand();
        const double thrust = nmpc.getAcc_zCommand();
        if (!rates.allFinite() || !std::isfinite(thrust))
        {
            ROS_ERROR_THROTTLE(1.0, "NMPC returned a non-finite command");
            publish_safe_attitude();
            return false;
        }

        attitude_setpoint.header.stamp = ros::Time::now();
        attitude_setpoint.header.frame_id = "FCU";
        attitude_setpoint.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        attitude_setpoint.body_rate.x = rates.x();
        attitude_setpoint.body_rate.y = rates.y();
        attitude_setpoint.body_rate.z = rates.z();
        attitude_setpoint.thrust = std::max(0.0, std::min(1.0, thrust));
        attitude_publisher.publish(attitude_setpoint);
        publish_nmpc_state(desired);

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
    const auto make_hold_reference = [&](const Eigen::Vector3d &position, double yaw) {
        std::vector<double> desired;
        const double half_yaw = 0.5 * yaw;
        for (int i = 0; i < 9; ++i)
        {
            desired.insert(desired.end(), {
                position.x(), position.y(), position.z(),
                0.0, 0.0, 0.0,
                std::cos(half_yaw), 0.0, 0.0, std::sin(half_yaw)});
        }
        for (int i = 0; i < 8; ++i)
        {
            desired.insert(desired.end(), {0.0, 0.0, 0.0, nmpc_gravity});
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
                    0.0, 0.0, point.yaw_dot, thrust_acceleration.norm()});
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
                0.0, 0.0, planner_command.yaw_dot, collective_acceleration});
        }
        run_nmpc(desired);
    };
    const auto nmpc_figure_eight = [&](int step) {
        const double current_phase = step * 0.02;
        update_xy_integral(Eigen::Vector2d(
            0.75 * std::sin(current_phase),
            0.375 * std::sin(2.0 * current_phase)));
        std::vector<double> desired;
        for (int i = 0; i < 9; ++i)
        {
            const double phase = step * 0.02 + i * nmpc.getNLPOnestepTime();
            desired.insert(desired.end(), {
                0.75 * std::sin(phase) + xy_integral_correction.x(),
                0.375 * std::sin(2.0 * phase) + xy_integral_correction.y(), 0.5,
                0.75 * std::cos(phase), 0.75 * std::cos(2.0 * phase), 0.0,
                1.0, 0.0, 0.0, 0.0});
        }
        for (int i = 0; i < 8; ++i)
        {
            desired.insert(desired.end(), {0.0, 0.0, 0.0, nmpc_gravity});
        }
        run_nmpc(desired);
    };

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
    int previous_command = 0;
    Eigen::Vector3d planner_fallback_hold = local_position;

    while (ros::ok())
    {
        ros::spinOnce();
        const int active_command = command.load(std::memory_order_relaxed);
        std_msgs::Int32 command_state;
        command_state.data = active_command;
        command_state_publisher.publish(command_state);
        if (active_command != previous_command)
        {
            trajectory_step = 0;
            reset_xy_integral();
            if (active_command == 5)
            {
                planner_fallback_hold = local_position;
                // A reference from the previous tracking session must not be reused
                // while the bridge processes the new start trigger.
                has_planner_command = false;
                has_planner_horizon = false;
            }
            if (active_command == 5 || previous_command == 5)
            {
                geometry_msgs::PoseStamped trigger;
                trigger.header.stamp = ros::Time::now();
                trigger.header.frame_id = active_command == 5 ? "start" : "stop";
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

        switch (active_command)
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
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);
            nmpc_hover();
            break;

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
            RequestOffboardAndArm(
                set_mode_client,
                arming_client,
                offboard_mode,
                arm_command,
                last_request);
            nmpc_planner_track(planner_fallback_hold);
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
                reference_pose.pose.position.z = 0.5;
                reference_pose.pose.orientation.w = 1.0;
                reference_velocity.twist.linear.x = 0.75 * std::cos(phase);
                reference_velocity.twist.linear.y = 0.75 * std::cos(2.0 * phase);
                trajectory_reference_publisher.publish(reference_pose);
                trajectory_reference_velocity_publisher.publish(reference_velocity);
            }
            trajectory_step = (trajectory_step + 1) % 315;
            break;

        default:
            break;
        }

        rate.sleep();
    }

    return 0;
}
