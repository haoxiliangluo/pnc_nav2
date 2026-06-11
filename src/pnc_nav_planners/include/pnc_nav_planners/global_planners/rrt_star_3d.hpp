// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_PLANNERS__GLOBAL_PLANNERS__RRT_STAR_3D_HPP_
#define PNC_NAV_PLANNERS__GLOBAL_PLANNERS__RRT_STAR_3D_HPP_

#include <memory>
#include <string>
#include <vector>
#include <random>

#include "pnc_nav_core/global_planner_base.hpp"

namespace pnc_nav_planners
{

/**
 * @struct RRTNode
 * @brief RRT树节点
 */
struct RRTNode
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double cost{0.0};        // 从根节点到此节点的代价
  int parent_id{-1};       // 父节点索引
};

/**
 * @class RRTStar3D
 * @brief RRT* 三维全局规划器
 *
 * 基于采样的渐近最优路径规划算法。
 * 支持 rewire 操作，随着采样数增加路径趋近最优。
 */
class RRTStar3D : public pnc_nav_core::GlobalPlannerBase
{
public:
  RRTStar3D() = default;
  ~RRTStar3D() override = default;

  void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name,
    const std::shared_ptr<pnc_nav_core::CostmapInterface> & costmap) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  std::string getName() const override { return "RRTStar3D"; }

private:
  // 随机采样
  RRTNode sampleRandom() const;

  // 找最近节点
  int findNearest(const RRTNode & sample) const;

  // 从 nearest 向 sample 方向扩展 step_size
  RRTNode steer(const RRTNode & nearest, const RRTNode & sample) const;

  // 碰撞检测（两点之间连线）
  bool isCollisionFree(const RRTNode & from, const RRTNode & to) const;

  // 找 rewire 半径内的邻居
  std::vector<int> findNearNodes(const RRTNode & node) const;

  // 计算两节点间距离
  double distance(const RRTNode & a, const RRTNode & b) const;

  // 回溯路径
  nav_msgs::msg::Path extractPath(int goal_id, const std_msgs::msg::Header & header) const;

  // --- 成员变量 ---
  rclcpp::Node::SharedPtr node_;
  std::string name_;
  std::shared_ptr<pnc_nav_core::CostmapInterface> costmap_;

  // RRT 树
  std::vector<RRTNode> tree_;

  // 参数
  int max_iterations_{50000};
  double step_size_{0.3};
  double goal_bias_{0.1};          // 目标偏向概率
  double rewire_radius_{0.5};
  double max_planning_time_{5.0};  // 秒
  double goal_tolerance_{0.2};

  // 采样范围
  double sample_min_x_{0.0}, sample_max_x_{0.0};
  double sample_min_y_{0.0}, sample_max_y_{0.0};
  double sample_min_z_{0.0}, sample_max_z_{0.0};

  // 随机数生成
  mutable std::mt19937 rng_;

  // 目标点缓存
  RRTNode goal_node_;
};

}  // namespace pnc_nav_planners

#endif  // PNC_NAV_PLANNERS__GLOBAL_PLANNERS__RRT_STAR_3D_HPP_
