#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <quadrotor_msgs/EstimatorState.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <string>

class LidarImuStateBridge
{
public:
    LidarImuStateBridge()
        : nh_(), pnh_("~")
    {
        pnh_.param("imu_timeout", imu_timeout_, 0.1);
        pnh_.param("velocity_timeout", velocity_timeout_, 0.1);
        pnh_.param("accel_bias_x", accel_bias_.x, 0.0);
        pnh_.param("accel_bias_y", accel_bias_.y, 0.0);
        pnh_.param("accel_bias_z", accel_bias_.z, 0.0);
        pnh_.param("gyro_bias_x", gyro_bias_.x, 0.0);
        pnh_.param("gyro_bias_y", gyro_bias_.y, 0.0);
        pnh_.param("gyro_bias_z", gyro_bias_.z, 0.0);

        std::string odom_topic = "/visual_slam/odom";
        std::string imu_topic = "/mavros/imu/data";
        std::string velocity_topic = "/mavros/local_position/velocity_local";
        std::string output_topic = "/estimator/state";
        pnh_.param("odom_topic", odom_topic, odom_topic);
        pnh_.param("imu_topic", imu_topic, imu_topic);
        pnh_.param("velocity_topic", velocity_topic, velocity_topic);
        pnh_.param("output_topic", output_topic, output_topic);

        output_pub_ = nh_.advertise<quadrotor_msgs::EstimatorState>(output_topic, 20);
        odom_sub_ = nh_.subscribe(odom_topic, 20, &LidarImuStateBridge::odomCallback, this,
                                  ros::TransportHints().tcpNoDelay());
        imu_sub_ = nh_.subscribe(imu_topic, 100, &LidarImuStateBridge::imuCallback, this,
                                 ros::TransportHints().tcpNoDelay());
        velocity_sub_ = nh_.subscribe(velocity_topic, 100, &LidarImuStateBridge::velocityCallback, this,
                                      ros::TransportHints().tcpNoDelay());
    }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr &message)
    {
        latest_imu_ = *message;
        has_imu_ = true;
    }

    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &message)
    {
        latest_velocity_ = *message;
        has_velocity_ = true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &message)
    {
        if (!has_imu_)
        {
            ROS_WARN_THROTTLE(1.0, "Lidar/IMU bridge is waiting for IMU data");
            return;
        }

        if (!has_velocity_)
        {
            ROS_WARN_THROTTLE(1.0, "Lidar/IMU bridge is waiting for MAVROS local velocity data");
            return;
        }

        const ros::Time now = ros::Time::now();
        if ((now - latest_imu_.header.stamp).toSec() > imu_timeout_)
        {
            ROS_WARN_THROTTLE(1.0, "Lidar/IMU bridge received stale IMU data");
            return;
        }
        if ((now - latest_velocity_.header.stamp).toSec() > velocity_timeout_)
        {
            ROS_WARN_THROTTLE(1.0, "Lidar/IMU bridge received stale MAVROS local velocity data");
            return;
        }

        quadrotor_msgs::EstimatorState output;
        output.header = message->header;
        output.header.frame_id = "world";
        output.attitude = message->pose.pose.orientation;
        output.velocity = latest_velocity_.twist.linear;
        output.unbiased_linear_acceleration.x =
            latest_imu_.linear_acceleration.x - accel_bias_.x;
        output.unbiased_linear_acceleration.y =
            latest_imu_.linear_acceleration.y - accel_bias_.y;
        output.unbiased_linear_acceleration.z =
            latest_imu_.linear_acceleration.z - accel_bias_.z;
        output.unbiased_angular_velocity.x =
            latest_imu_.angular_velocity.x - gyro_bias_.x;
        output.unbiased_angular_velocity.y =
            latest_imu_.angular_velocity.y - gyro_bias_.y;
        output.unbiased_angular_velocity.z =
            latest_imu_.angular_velocity.z - gyro_bias_.z;
        output_pub_.publish(output);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber velocity_sub_;
    ros::Publisher output_pub_;
    sensor_msgs::Imu latest_imu_;
    geometry_msgs::TwistStamped latest_velocity_;
    geometry_msgs::Vector3 accel_bias_;
    geometry_msgs::Vector3 gyro_bias_;
    bool has_imu_{false};
    bool has_velocity_{false};
    double imu_timeout_{0.1};
    double velocity_timeout_{0.1};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_imu_state_bridge");
    LidarImuStateBridge bridge;
    ros::spin();
    return 0;
}
