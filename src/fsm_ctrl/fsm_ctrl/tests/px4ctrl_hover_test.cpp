#include <fsm_ctrl/px4ctrl_hover.hpp>
#include <gtest/gtest.h>
#include <cmath>

class HoverTest : public ::testing::Test
{
protected:
    std::unique_ptr<Px4CtrlHover> hover;
    geometry_msgs::PoseStamped::Ptr pose;
    geometry_msgs::TwistStamped::Ptr velocity;
    sensor_msgs::Imu::Ptr imu;
    geometry_msgs::Pose target;
    double hover_percentage = 0.0;

    void SetUp() override
    {
        ros::NodeHandle params("~controller");
        hover.reset(new Px4CtrlHover(params));
        params.getParam("thrust_model/hover_percentage", hover_percentage);
        pose.reset(new geometry_msgs::PoseStamped);
        velocity.reset(new geometry_msgs::TwistStamped);
        imu.reset(new sensor_msgs::Imu);
        const auto now = ros::Time::now();
        pose->header.stamp = velocity->header.stamp = imu->header.stamp = now;
        pose->pose.position.z = 0.4;
        pose->pose.orientation.w = imu->orientation.w = 1.0;
        params.getParam("gra", imu->linear_acceleration.z);
        target = pose->pose;
    }
    void feed()
    {
        hover->poseCallback(pose);
        hover->velocityCallback(velocity);
        hover->imuCallback(imu);
    }
};

TEST_F(HoverTest, RequiresAllFeedback)
{
    EXPECT_FALSE(hover->ready(ros::Time::now()));
    hover->poseCallback(pose);
    hover->imuCallback(imu);
    EXPECT_FALSE(hover->ready(ros::Time::now()));
    hover->velocityCallback(velocity);
    EXPECT_TRUE(hover->ready(ros::Time::now()));
}

TEST_F(HoverTest, RejectsOldAndFutureMeasurements)
{
    feed();
    velocity->header.stamp = ros::Time::now() - ros::Duration(2.0);
    hover->velocityCallback(velocity);
    EXPECT_FALSE(hover->ready(ros::Time::now()));
    velocity->header.stamp = ros::Time::now() + ros::Duration(2.0);
    hover->velocityCallback(velocity);
    EXPECT_FALSE(hover->ready(ros::Time::now()));
}

TEST_F(HoverTest, LevelHoverUsesOriginalThrustMapping)
{
    feed();
    mavros_msgs::AttitudeTarget output;
    quadrotor_msgs::Px4ctrlDebug debug;
    ASSERT_TRUE(hover->calculate(target, ros::Time::now(), false, output, debug));
    EXPECT_NEAR(output.thrust, hover_percentage, 1e-6);
    EXPECT_NEAR(output.orientation.w, 1.0, 1e-6);
    EXPECT_NEAR(output.orientation.x, 0.0, 1e-6);
    EXPECT_NEAR(output.orientation.y, 0.0, 1e-6);
    EXPECT_EQ(output.type_mask, 7);
    EXPECT_NEAR(debug.des_p_z, 0.4, 1e-6);
}

TEST_F(HoverTest, PositionErrorCreatesCorrectiveAttitude)
{
    pose->pose.position.x = 0.2;
    feed();
    mavros_msgs::AttitudeTarget output;
    quadrotor_msgs::Px4ctrlDebug debug;
    ASSERT_TRUE(hover->calculate(target, ros::Time::now(), false, output, debug));
    EXPECT_LT(output.orientation.y, 0.0);
    EXPECT_NEAR(debug.des_p_x, 0.0, 1e-6);
}

TEST_F(HoverTest, VelocityFeedbackIsInWorldFrame)
{
    // +world-X velocity must brake towards -world-X even at 90 degree yaw.
    pose->pose.orientation.w = pose->pose.orientation.z = std::sqrt(0.5);
    imu->orientation = pose->pose.orientation;
    velocity->twist.linear.x = 0.2;
    feed();
    mavros_msgs::AttitudeTarget output;
    quadrotor_msgs::Px4ctrlDebug debug;
    ASSERT_TRUE(hover->calculate(target, ros::Time::now(), false, output, debug));
    EXPECT_LT(debug.des_a_x, 0.0);
    EXPECT_NEAR(debug.des_a_y, 0.0, 1e-6);
}

TEST_F(HoverTest, InvalidQuaternionInvalidatesFeedback)
{
    feed();
    pose->pose.orientation.w = 0.0;
    hover->poseCallback(pose);
    EXPECT_FALSE(hover->ready(ros::Time::now()));
}

TEST_F(HoverTest, IdleMatchesBaselineLowThrust)
{
    feed();
    auto output = hover->idle(ros::Time::now());
    EXPECT_NEAR(output.thrust, 0.02, 1e-6);
    EXPECT_NEAR(output.orientation.w, 1.0, 1e-6);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4ctrl_hover_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
