// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_task/mission_runner.hpp"

namespace pnc_nav_task
{

const char * toString(MissionState state)
{
  switch (state) {
    case MissionState::IDLE: return "idle";
    case MissionState::RUNNING: return "running";
    case MissionState::PAUSED: return "paused";
    case MissionState::SUCCEEDED: return "succeeded";
    case MissionState::FAILED: return "failed";
    case MissionState::CANCELLED: return "cancelled";
  }
  return "unknown";
}

MissionRunner::MissionRunner(MissionConfig config)
: config_(std::move(config))
{
}

bool MissionRunner::finished() const
{
  return state_ == MissionState::SUCCEEDED ||
         state_ == MissionState::FAILED ||
         state_ == MissionState::CANCELLED;
}

void MissionRunner::start()
{
  if (config_.waypoints.empty()) {
    state_ = MissionState::FAILED;
    message_ = "no waypoints configured";
    return;
  }
  state_ = MissionState::RUNNING;
  index_ = 0;
  retries_ = 0;
  message_ = "started";
}

void MissionRunner::pause()
{
  if (state_ == MissionState::RUNNING) {
    state_ = MissionState::PAUSED;
    message_ = "paused";
  }
}

void MissionRunner::resume()
{
  if (state_ == MissionState::PAUSED) {
    state_ = MissionState::RUNNING;
    message_ = "resumed";
  }
}

void MissionRunner::cancel()
{
  state_ = MissionState::CANCELLED;
  message_ = "cancelled by user";
}

void MissionRunner::onGoalSucceeded()
{
  if (state_ != MissionState::RUNNING) {
    return;
  }
  retries_ = 0;
  if (index_ + 1 < config_.waypoints.size()) {
    ++index_;
    message_ = "waypoint reached, next";
    return;
  }
  if (config_.loop) {
    index_ = 0;
    message_ = "waypoint reached, loop";
    return;
  }
  state_ = MissionState::SUCCEEDED;
  message_ = "all waypoints reached";
}

void MissionRunner::onGoalFailed()
{
  if (state_ != MissionState::RUNNING) {
    return;
  }
  if (retries_ < config_.max_retries) {
    ++retries_;
    message_ = "waypoint failed, retry";
    return;
  }
  state_ = MissionState::FAILED;
  message_ = "waypoint failed, retries exhausted";
}

geometry_msgs::msg::PoseStamped MissionRunner::currentWaypoint() const
{
  if (index_ < config_.waypoints.size()) {
    return config_.waypoints[index_];
  }
  return geometry_msgs::msg::PoseStamped{};
}

std::string MissionRunner::message() const
{
  return message_;
}

}  // namespace pnc_nav_task
