// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_PLANNERS__GLOBAL_PLANNERS__ASTAR_3D_HPP_
#define PNC_NAV_PLANNERS__GLOBAL_PLANNERS__ASTAR_3D_HPP_

#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>

#include "pnc_nav_core/global_planner_base.hpp"

namespace pnc_nav_planners
{

/**
 * @struct GridIndex
 * @brief 3D栅格索引
 */
struct GridIndex
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

/// GridIndex 哈希函数
struct GridIndexHash
{
  size_t operator()(const GridIndex & idx) const
  {
    size_t h1 = std::hash<int>()(idx.x);
    size_t h2 = std::hash<int>()(idx.y);
    size_t h3 = std::hash<int>()(idx.z);
    return h1 ^ (h2 << 16) ^ (h3 << 32);
  }
};

/**
 * @struct AStarNode
 * @brief A*搜索节点
 */
struct AStarNode
{
  GridIndex index;
  double g_cost{0.0};    // 起点到当前的代价
  double h_cost{0.0};    // 启发式估计（当前到终点）
  double f_cost{0.0};    // g + h
  GridIndex parent;
  bool in_open{false};
  bool in_closed{false};

  bool operator>(const AStarNode & other) const
  {
    return f_cost > other.f_cost;
  }
};

/**
 * @class AStar3D
 * @brief A* 三维全局规划器
 *
 * 支持在 OctoMap 或 2D OccupancyGrid 上进行路径搜索。
 * 2D 模式下忽略 z 轴，3D 模式下支持 26 邻域扩展。
 */
class AStar3D : public pnc_nav_core::GlobalPlannerBase
{
public:
  AStar3D() = default;
  ~AStar3D() override = default;

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

  std::string getName() const override { return "AStar3D"; }

private:
  // 世界坐标 → 栅格索引
  GridIndex worldToGrid(double x, double y, double z) const;
  // 栅格索引 → 世界坐标
  void gridToWorld(const GridIndex & idx, double & x, double & y, double & z) const;

  // 启发式函数
  double heuristic(const GridIndex & a, const GridIndex & b) const;

  // 获取邻居节点
  std::vector<GridIndex> getNeighbors(const GridIndex & current) const;

  // 回溯路径
  nav_msgs::msg::Path reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    const GridIndex & start,
    const GridIndex & goal,
    const std_msgs::msg::Header & header) const;

  // 路径平滑
  nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path & raw_path) const;

  // --- 成员变量 ---
  rclcpp::Node::SharedPtr node_;// ROS节点指针
  std::string name_; // 规划器实例名称
  std::shared_ptr<pnc_nav_core::CostmapInterface> costmap_;// 代价地图接口

  // 参数
  double resolution_{0.1};// 栅格分辨率 (m)
  double heuristic_weight_{1.2};// 启发式权重
  int max_iterations_{100000};// 最大迭代次数
  bool allow_unknown_{false};// 是否允许未知区域
  bool diagonal_movement_{true};// 是否允许对角移动
  double height_penalty_{2.0};// 高度惩罚系数 (3D模式下)
  bool use_3d_{false};           // false = 2D模式, true = 3D模式

  // 地图边界（栅格坐标）
  int grid_min_x_{0}, grid_max_x_{0}; // 2D模式下z轴固定为0
  int grid_min_y_{0}, grid_max_y_{0};// 3D模式下z轴范围
  int grid_min_z_{0}, grid_max_z_{0};// 3D模式下z轴范围
};

}  // namespace pnc_nav_planners

#endif  // PNC_NAV_PLANNERS__GLOBAL_PLANNERS__ASTAR_3D_HPP_
