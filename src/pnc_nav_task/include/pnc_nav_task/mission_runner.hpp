// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_TASK__MISSION_RUNNER_HPP_
#define PNC_NAV_TASK__MISSION_RUNNER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pnc_nav_task
{

/// 任务状态（与 MissionStatus.msg 的常量一一对应）
enum class MissionState : uint8_t
{
  IDLE = 0,
  RUNNING = 1,
  PAUSED = 2,
  SUCCEEDED = 3,
  FAILED = 4,
  CANCELLED = 5,
};

struct MissionConfig
{
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  int max_retries{1};   // 每个点失败后的重试次数（总尝试 = 1 + max_retries）
  bool loop{false};     // 跑完是否从头再来
};

/**
 * @class MissionRunner
 * @brief 任务层的纯逻辑（不碰 ROS）：顺序推进 + 重试 + 暂停/恢复/取消
 *
 * 设计意图：把"任务该怎么走"和"怎么调 action"分开，前者可以单元测试，
 * 后者（mission_executor_node）只负责收发。
 */
class MissionRunner
{
public:
  explicit MissionRunner(MissionConfig config);

  // --- 命令 ---
  void start();    // 任意状态 → RUNNING（从头开始）
  void pause();    // RUNNING → PAUSED
  void resume();   // PAUSED → RUNNING
  void cancel();   // 任意状态 → CANCELLED

  // --- 导航结果回调（由节点在 action 结束时调用）---
  void onGoalSucceeded();
  void onGoalFailed();

  // --- 查询 ---
  MissionState state() const {return state_;}
  bool finished() const;
  bool running() const {return state_ == MissionState::RUNNING;}
  bool needsGoal() const {return state_ == MissionState::RUNNING;}// 当前点需要发目标
  size_t currentIndex() const {return index_;}
  size_t total() const {return config_.waypoints.size();}
  int retriesDone() const {return retries_;}
  const MissionConfig & config() const {return config_;}
  geometry_msgs::msg::PoseStamped currentWaypoint() const;
  std::string message() const;

private:
  MissionConfig config_;
  MissionState state_{MissionState::IDLE};
  size_t index_{0};
  int retries_{0};
  std::string message_{"idle"};
};

const char * toString(MissionState state);

}  // namespace pnc_nav_task

#endif  // PNC_NAV_TASK__MISSION_RUNNER_HPP_
