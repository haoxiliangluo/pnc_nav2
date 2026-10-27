// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include <gtest/gtest.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "pnc_nav_locomotion/fake_trot_locomotion.hpp"

using pnc_nav_core::RobotState;

namespace
{

class FakeTrotTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_fake_trot");
    trot_.configure(node_, "fake_trot");
  }

  rclcpp::Node::SharedPtr node_;
  pnc_nav_locomotion::FakeTrotLocomotion trot_;
};

}  // namespace

TEST_F(FakeTrotTest, ZeroVelocityGivesNeutralJoints)
{
  const RobotState state;
  const auto cmd = trot_.computeJointCommands(geometry_msgs::msg::Twist{}, state);
  EXPECT_EQ(cmd.positions.size(), 12u);
  EXPECT_NEAR(trot_.lastAmplitude(), 0.0, 1e-9);
  for (double p : cmd.positions) {
    EXPECT_NEAR(p, 0.0, 1e-9);
  }
}

TEST_F(FakeTrotTest, AmplitudeGrowsWithSpeed)
{
  RobotState state;
  geometry_msgs::msg::Twist slow;
  slow.linear.x = 0.1;
  trot_.computeJointCommands(slow, state);
  const double amp_slow = trot_.lastAmplitude();

  geometry_msgs::msg::Twist fast;
  fast.linear.x = 0.3;
  trot_.computeJointCommands(fast, state);
  const double amp_fast = trot_.lastAmplitude();

  EXPECT_GT(amp_slow, 0.0);
  EXPECT_GT(amp_fast, amp_slow);
}

TEST_F(FakeTrotTest, LegsMoveWhenDriving)
{
  RobotState state;
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.3;
  const double a = trot_.computeJointCommands(cmd, state).positions[1];
  const double b = trot_.computeJointCommands(cmd, state).positions[1];
  EXPECT_NE(a, b);           // 相位在推进，大腿关节在动
}

TEST_F(FakeTrotTest, EmergencyStopReturnsNeutral)
{
  RobotState state;
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.3;
  trot_.computeJointCommands(cmd, state);
  const auto stop = trot_.emergencyStop(state);
  EXPECT_NEAR(trot_.lastAmplitude(), 0.0, 1e-9);
  for (double p : stop.positions) {
    EXPECT_NEAR(p, 0.0, 1e-9);
  }
}

TEST_F(FakeTrotTest, ControlFrequencyIsConfigured)
{
  EXPECT_GT(trot_.getControlFrequency(), 10.0);
}
