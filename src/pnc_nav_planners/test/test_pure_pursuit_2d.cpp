// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include <gtest/gtest.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "pnc_nav_planners/path_trackers/pure_pursuit_2d.hpp"

namespace
{
nav_msgs::msg::Path makeStraightPath(double x0, double x1, double y = 0.0, double step = 0.1)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const double sign = (x1 >= x0) ? 1.0 : -1.0;
  for (double x = x0; sign * (x1 - x) >= -1e-9; x += sign * step) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}
}  // namespace

TEST(PurePursuit2D, DrivesForwardOnForwardPath)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_pp_forward");
  pnc_nav_planners::PurePursuit2D tracker;
  tracker.configure(node, "path_tracker");
  ASSERT_TRUE(tracker.setPath(makeStraightPath(0.0, 2.0)));

  const auto cmd = tracker.computeVelocityCommand(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist{});
  EXPECT_GT(cmd.twist.linear.x, 0.0);
  rclcpp::shutdown();
}

TEST(PurePursuit2D, RotatesInPlaceWhenPathIsBehind)   // KI-020 回归
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_pp_reverse");
  pnc_nav_planners::PurePursuit2D tracker;
  tracker.configure(node, "path_tracker");
  // 路径在机器人身后（从原点往 -x 方向）
  ASSERT_TRUE(tracker.setPath(makeStraightPath(0.0, -1.5)));

  const auto cmd = tracker.computeVelocityCommand(makePose(0.0, 0.0, 0.0), geometry_msgs::msg::Twist{});
  EXPECT_NEAR(cmd.twist.linear.x, 0.0, 1e-9);          // 不前进
  EXPECT_GT(std::abs(cmd.twist.angular.z), 0.1);       // 原地转
  rclcpp::shutdown();
}
