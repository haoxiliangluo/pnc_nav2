// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include <gtest/gtest.h>

#include "pnc_nav_task/mission_runner.hpp"

using pnc_nav_task::MissionConfig;
using pnc_nav_task::MissionRunner;
using pnc_nav_task::MissionState;

namespace
{
MissionConfig makeConfig(size_t n, int max_retries = 1, bool loop = false)
{
  MissionConfig config;
  config.max_retries = max_retries;
  config.loop = loop;
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    config.waypoints.push_back(pose);
  }
  return config;
}
}  // namespace

TEST(MissionRunner, StartRequiresWaypoints)
{
  MissionRunner empty{MissionConfig{}};
  empty.start();
  EXPECT_EQ(empty.state(), MissionState::FAILED);
  EXPECT_TRUE(empty.finished());
}

TEST(MissionRunner, RunsAllWaypointsInOrder)
{
  MissionRunner runner{makeConfig(3)};
  runner.start();
  ASSERT_TRUE(runner.running());
  EXPECT_EQ(runner.currentIndex(), 0u);

  runner.onGoalSucceeded();
  EXPECT_EQ(runner.currentIndex(), 1u);
  runner.onGoalSucceeded();
  EXPECT_EQ(runner.currentIndex(), 2u);
  runner.onGoalSucceeded();
  EXPECT_EQ(runner.state(), MissionState::SUCCEEDED);
  EXPECT_TRUE(runner.finished());
}

TEST(MissionRunner, LoopWrapsAround)
{
  MissionRunner runner{makeConfig(2, 0, true)};
  runner.start();
  runner.onGoalSucceeded();   // → index 1
  runner.onGoalSucceeded();   // loop → index 0
  EXPECT_EQ(runner.state(), MissionState::RUNNING);
  EXPECT_EQ(runner.currentIndex(), 0u);
}

TEST(MissionRunner, RetriesThenFails)
{
  MissionRunner runner{makeConfig(2, 2)};
  runner.start();
  runner.onGoalFailed();      // retry 1
  EXPECT_EQ(runner.state(), MissionState::RUNNING);
  EXPECT_EQ(runner.retriesDone(), 1);
  runner.onGoalFailed();      // retry 2
  EXPECT_EQ(runner.retriesDone(), 2);
  runner.onGoalFailed();      // 超过重试上限
  EXPECT_EQ(runner.state(), MissionState::FAILED);
}

TEST(MissionRunner, SuccessResetsRetries)
{
  MissionRunner runner{makeConfig(2, 2)};
  runner.start();
  runner.onGoalFailed();
  EXPECT_EQ(runner.retriesDone(), 1);
  runner.onGoalSucceeded();
  EXPECT_EQ(runner.retriesDone(), 0);
  EXPECT_EQ(runner.currentIndex(), 1u);
}

TEST(MissionRunner, PauseResumeAndCancel)
{
  MissionRunner runner{makeConfig(2)};
  runner.start();
  runner.pause();
  EXPECT_EQ(runner.state(), MissionState::PAUSED);
  EXPECT_FALSE(runner.needsGoal());
  runner.onGoalSucceeded();   // 暂停期间的结果不推进
  EXPECT_EQ(runner.currentIndex(), 0u);

  runner.resume();
  EXPECT_TRUE(runner.running());
  EXPECT_TRUE(runner.needsGoal());

  runner.cancel();
  EXPECT_EQ(runner.state(), MissionState::CANCELLED);
  EXPECT_TRUE(runner.finished());
  runner.onGoalSucceeded();
  EXPECT_EQ(runner.state(), MissionState::CANCELLED);
}

TEST(MissionRunner, CurrentWaypointMatchesIndex)
{
  MissionRunner runner{makeConfig(3)};
  runner.start();
  EXPECT_DOUBLE_EQ(runner.currentWaypoint().pose.position.x, 0.0);
  runner.onGoalSucceeded();
  EXPECT_DOUBLE_EQ(runner.currentWaypoint().pose.position.x, 1.0);
}
