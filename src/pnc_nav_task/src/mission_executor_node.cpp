// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License
//
// mission_executor_node：任务层骨架
// - 顺序执行配置里的 waypoints（每点可重试 N 次）
// - 服务：mission/start | pause | resume | cancel（std_srvs/Trigger）
// - 话题：/mission_status（pnc_nav_interfaces/MissionStatus，2Hz + 状态变化即发）

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "pnc_nav_interfaces/action/navigate_to3_d_pose.hpp"
#include "pnc_nav_interfaces/msg/mission_status.hpp"
#include "pnc_nav_task/mission_runner.hpp"

namespace pnc_nav_task
{

class MissionExecutorNode : public rclcpp::Node
{
public:
  using NavigateTo3DPose = pnc_nav_interfaces::action::NavigateTo3DPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateTo3DPose>;

  MissionExecutorNode() : Node("mission_executor")
  {
    MissionConfig config;
    config.max_retries = declare_parameter<int>("mission.max_retries", 1);
    config.loop = declare_parameter<bool>("mission.loop", false);
    const std::string frame_id = declare_parameter<std::string>("mission.frame_id", "map");
    auto_start_ = declare_parameter<bool>("mission.auto_start", false);
    const double tick_period = declare_parameter<double>("mission.tick_period", 0.2);

    const auto flat = declare_parameter<std::vector<double>>(
      "mission.waypoints", std::vector<double>{});
    if (flat.size() % 2 != 0) {
      RCLCPP_WARN(get_logger(), "mission.waypoints 长度不是偶数，最后一个值被忽略");
    }
    for (size_t i = 0; i + 1 < flat.size(); i += 2) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_id;
      pose.pose.position.x = flat[i];
      pose.pose.position.y = flat[i + 1];
      pose.pose.orientation.w = 1.0;
      config.waypoints.push_back(pose);
    }

    runner_ = std::make_unique<MissionRunner>(config);
    client_ = rclcpp_action::create_client<NavigateTo3DPose>(this, "navigate_to_3d_pose");

    status_pub_ = create_publisher<pnc_nav_interfaces::msg::MissionStatus>("mission_status", 1);

    using Trigger = std_srvs::srv::Trigger;
    start_srv_ = create_service<Trigger>(
      "mission/start",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        runner_->start();
        res->success = runner_->running();
        res->message = runner_->message();
        publishStatus();
      });
    pause_srv_ = create_service<Trigger>(
      "mission/pause",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        runner_->pause();
        cancelActiveGoal();
        res->success = runner_->state() == MissionState::PAUSED;
        res->message = runner_->message();
        publishStatus();
      });
    resume_srv_ = create_service<Trigger>(
      "mission/resume",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        runner_->resume();
        res->success = runner_->running();
        res->message = runner_->message();
        publishStatus();
      });
    cancel_srv_ = create_service<Trigger>(
      "mission/cancel",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        runner_->cancel();
        cancelActiveGoal();
        res->success = true;
        res->message = runner_->message();
        publishStatus();
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.05, tick_period)),
      std::bind(&MissionExecutorNode::tick, this));
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&MissionExecutorNode::publishStatus, this));

    RCLCPP_INFO(
      get_logger(), "mission_executor ready: %zu waypoints, max_retries=%d, loop=%s, auto_start=%s",
      runner_->total(), config.max_retries, config.loop ? "true" : "false",
      auto_start_ ? "true" : "false");

    if (auto_start_) {
      runner_->start();
    }
  }

private:
  void tick()
  {
    if (!runner_->needsGoal() || goal_inflight_) {
      return;
    }
    if (!client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "等待 navigate_to_3d_pose action server ...");
      return;
    }

    NavigateTo3DPose::Goal goal;
    goal.goal = runner_->currentWaypoint();
    goal.planner_id = "";
    goal.max_planning_time = 0.0;

    rclcpp_action::Client<NavigateTo3DPose>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
        if (!handle) {
          RCLCPP_WARN(get_logger(), "目标被拒绝");
          goal_inflight_ = false;
          runner_->onGoalFailed();
          publishStatus();
          return;
        }
        goal_handle_ = handle;
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        onGoalResult(result);
      };

    goal_inflight_ = true;
    client_->async_send_goal(goal, options);
    RCLCPP_INFO(
      get_logger(), "发送第 %zu/%zu 个目标 (%.2f, %.2f)", runner_->currentIndex() + 1,
      runner_->total(), goal.goal.pose.position.x, goal.goal.pose.position.y);
  }

  void onGoalResult(const GoalHandle::WrappedResult & result);
  void cancelActiveGoal();
  void publishStatus();

  std::unique_ptr<MissionRunner> runner_;
  rclcpp_action::Client<NavigateTo3DPose>::SharedPtr client_;
  std::shared_ptr<GoalHandle> goal_handle_;
  bool goal_inflight_{false};
  bool auto_start_{false};
  rclcpp::Publisher<pnc_nav_interfaces::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

void MissionExecutorNode::onGoalResult(const GoalHandle::WrappedResult & result)
{
  goal_inflight_ = false;
  goal_handle_.reset();

  const bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED) &&
    result.result && result.result->success;
  const uint16_t error_code = result.result ? result.result->error_code : 0;

  RCLCPP_INFO(
    get_logger(), "目标结束: code=%d success=%s error_code=%u msg=%s",
    static_cast<int>(result.code), success ? "true" : "false", error_code,
    result.result ? result.result->message.c_str() : "");

  if (runner_->state() != MissionState::RUNNING) {
    // 暂停/取消期间返回的结果不推进任务
    publishStatus();
    return;
  }

  if (success) {
    runner_->onGoalSucceeded();
  } else {
    runner_->onGoalFailed();
  }
  publishStatus();
}

void MissionExecutorNode::cancelActiveGoal()
{
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }
}

void MissionExecutorNode::publishStatus()
{
  pnc_nav_interfaces::msg::MissionStatus msg;
  msg.header.stamp = now();
  msg.header.frame_id = runner_->config().waypoints.empty()
    ? "map" : runner_->currentWaypoint().header.frame_id;
  msg.state = static_cast<uint8_t>(runner_->state());
  msg.current_index = static_cast<uint16_t>(runner_->currentIndex() + 1);
  msg.total = static_cast<uint16_t>(runner_->total());
  msg.retries_done = static_cast<uint16_t>(runner_->retriesDone());
  msg.current_waypoint = runner_->currentWaypoint();
  msg.message = runner_->message();
  status_pub_->publish(msg);
}

}  // namespace pnc_nav_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pnc_nav_task::MissionExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
