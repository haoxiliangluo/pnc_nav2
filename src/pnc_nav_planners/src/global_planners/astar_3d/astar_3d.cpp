// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_planners/global_planners/astar_3d.hpp"

#include <cmath>
#include <algorithm>

#include "pluginlib/class_list_macros.hpp"

namespace pnc_nav_planners
{

void AStar3D::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const std::shared_ptr<pnc_nav_core::CostmapInterface> & costmap)
{
  node_ = node;
  name_ = name;
  costmap_ = costmap;

  // 从参数服务器获取配置
  node_->declare_parameter(name_ + ".resolution", 0.1);
  node_->declare_parameter(name_ + ".heuristic_weight", 1.2);
  node_->declare_parameter(name_ + ".max_iterations", 100000);
  node_->declare_parameter(name_ + ".allow_unknown", false);
  node_->declare_parameter(name_ + ".diagonal_movement", true);
  node_->declare_parameter(name_ + ".height_penalty", 2.0);
  node_->declare_parameter(name_ + ".use_3d", false);

  resolution_ = node_->get_parameter(name_ + ".resolution").as_double();
  heuristic_weight_ = node_->get_parameter(name_ + ".heuristic_weight").as_double();
  max_iterations_ = node_->get_parameter(name_ + ".max_iterations").as_int();
  allow_unknown_ = node_->get_parameter(name_ + ".allow_unknown").as_bool();
  diagonal_movement_ = node_->get_parameter(name_ + ".diagonal_movement").as_bool();
  height_penalty_ = node_->get_parameter(name_ + ".height_penalty").as_double();
  use_3d_ = node_->get_parameter(name_ + ".use_3d").as_bool();

  RCLCPP_INFO(node_->get_logger(), "AStar3D configured: res=%.2f, 3d=%s",
    resolution_, use_3d_ ? "true" : "false");
}

void AStar3D::activate()
{
  RCLCPP_INFO(node_->get_logger(), "AStar3D activated");
}

void AStar3D::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "AStar3D deactivated");
}

void AStar3D::cleanup()
{
  node_.reset();
  costmap_.reset();
}

GridIndex AStar3D::worldToGrid(double x, double y, double z) const
{
  GridIndex idx;
  idx.x = static_cast<int>(std::floor(x / resolution_));
  idx.y = static_cast<int>(std::floor(y / resolution_));
  idx.z = use_3d_ ? static_cast<int>(std::floor(z / resolution_)) : 0;
  return idx;
}

void AStar3D::gridToWorld(const GridIndex & idx, double & x, double & y, double & z) const
{
  x = (idx.x + 0.5) * resolution_;
  y = (idx.y + 0.5) * resolution_;
  z = use_3d_ ? (idx.z + 0.5) * resolution_ : 0.0;
}

double AStar3D::heuristic(const GridIndex & a, const GridIndex & b) const
{
  double dx = std::abs(a.x - b.x);
  double dy = std::abs(a.y - b.y);
  double dz = std::abs(a.z - b.z) * height_penalty_;

  if (diagonal_movement_) {
    // Octile distance (3D)
    double dmin = std::min({dx, dy, dz});
    double dmax = std::max({dx, dy, dz});
    double dmid = dx + dy + dz - dmin - dmax;
    return (std::sqrt(3.0) - std::sqrt(2.0)) * dmin +
           (std::sqrt(2.0) - 1.0) * dmid + dmax;
  } else {
    // Manhattan distance
    return dx + dy + dz;
  }
}

std::vector<GridIndex> AStar3D::getNeighbors(const GridIndex & current) const
{
  std::vector<GridIndex> neighbors;

  if (use_3d_) {
    // 26-邻域 (3D)
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          if (!diagonal_movement_ && (std::abs(dx) + std::abs(dy) + std::abs(dz) > 1)) continue;

          GridIndex n{current.x + dx, current.y + dy, current.z + dz};
          neighbors.push_back(n);
        }
      }
    }
  } else {
    // 8-邻域 (2D)
    const int dirs[][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    int num_dirs = diagonal_movement_ ? 8 : 4;
    for (int i = 0; i < num_dirs; ++i) {
      GridIndex n{current.x + dirs[i][0], current.y + dirs[i][1], 0};
      neighbors.push_back(n);
    }
  }

  return neighbors;
}

nav_msgs::msg::Path AStar3D::reconstructPath(
  const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
  const GridIndex & start,
  const GridIndex & goal,
  const std_msgs::msg::Header & header) const
{
  nav_msgs::msg::Path path;
  path.header = header;

  GridIndex current = goal;
  std::vector<GridIndex> indices;

  while (!(current == start)) {
    indices.push_back(current);
    auto it = came_from.find(current);
    if (it == came_from.end()) break;
    current = it->second;
  }
  indices.push_back(start);

  // 反转得到从起点到终点的路径
  std::reverse(indices.begin(), indices.end());

  for (const auto & idx : indices) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    gridToWorld(idx, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

nav_msgs::msg::Path AStar3D::smoothPath(const nav_msgs::msg::Path & raw_path) const
{
  // 简单的路径平滑：移动平均
  if (raw_path.poses.size() < 3) return raw_path;

  nav_msgs::msg::Path smooth = raw_path;
  const int window = 3;

  for (size_t i = 1; i < smooth.poses.size() - 1; ++i) {
    double sx = 0.0, sy = 0.0, sz = 0.0;
    int count = 0;
    for (int j = -window/2; j <= window/2; ++j) {
      int idx = static_cast<int>(i) + j;
      if (idx >= 0 && idx < static_cast<int>(raw_path.poses.size())) {
        sx += raw_path.poses[idx].pose.position.x;
        sy += raw_path.poses[idx].pose.position.y;
        sz += raw_path.poses[idx].pose.position.z;
        count++;
      }
    }
    smooth.poses[i].pose.position.x = sx / count;
    smooth.poses[i].pose.position.y = sy / count;
    smooth.poses[i].pose.position.z = sz / count;
  }

  return smooth;
}

nav_msgs::msg::Path AStar3D::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header = start.header;

  if (!costmap_) {
    RCLCPP_ERROR(node_->get_logger(), "AStar3D: costmap not available");
    return empty_path;
  }

  // 转换为栅格坐标
  GridIndex start_idx = worldToGrid(
    start.pose.position.x, start.pose.position.y, start.pose.position.z);
  GridIndex goal_idx = worldToGrid(
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

  // 检查起点和终点是否可通行
  if (costmap_->isOccupied(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)) {
    RCLCPP_WARN(node_->get_logger(), "AStar3D: goal is in obstacle");
    return empty_path;
  }

  // A* 搜索
  using PQElement = std::pair<double, GridIndex>;
  auto cmp = [](const PQElement & a, const PQElement & b) { return a.first > b.first; };
  std::priority_queue<PQElement, std::vector<PQElement>, decltype(cmp)> open_set(cmp);// 

  std::unordered_map<GridIndex, double, GridIndexHash> g_score;// 记录从起点到各节点的已知最小代价
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;// 记录路径回溯信息
  std::unordered_map<GridIndex, bool, GridIndexHash> closed_set;// 记录已访问节点

  g_score[start_idx] = 0.0;
  open_set.push({heuristic_weight_ * heuristic(start_idx, goal_idx), start_idx});

  int iterations = 0; // 迭代计数器

  while (!open_set.empty() && iterations < max_iterations_) {
    iterations++;

    auto [f, current] = open_set.top();
    open_set.pop();

    // 到达目标
    if (current == goal_idx) {
      RCLCPP_INFO(node_->get_logger(), "AStar3D: path found in %d iterations", iterations);
      auto raw_path = reconstructPath(came_from, start_idx, goal_idx, start.header);
      return smoothPath(raw_path);
    }

    if (closed_set.count(current)) continue;
    closed_set[current] = true;

    // 扩展邻居
    for (const auto & neighbor : getNeighbors(current)) {
      if (closed_set.count(neighbor)) continue;

      // 检查是否可通行
      double wx, wy, wz;
      gridToWorld(neighbor, wx, wy, wz);

      if (!costmap_->isInBounds(wx, wy, wz)) continue;
      if (costmap_->isOccupied(wx, wy, wz)) continue;
      if (!allow_unknown_ && costmap_->getCost(wx, wy, wz) == pnc_nav_core::cost_values::UNKNOWN) {
        continue;
      }

      // 计算移动代价
      double dx = std::abs(neighbor.x - current.x);
      double dy = std::abs(neighbor.y - current.y);
      double dz = std::abs(neighbor.z - current.z);
      double move_cost = std::sqrt(dx*dx + dy*dy + (dz * height_penalty_) * (dz * height_penalty_));

      // 加上代价地图的代价
      uint8_t cell_cost = costmap_->getCost(wx, wy, wz);
      double cost_factor = 1.0 + static_cast<double>(cell_cost) / 252.0;

      double tentative_g = g_score[current] + move_cost * cost_factor;// 计算新的g值

      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current;
        double f_score = tentative_g + heuristic_weight_ * heuristic(neighbor, goal_idx);
        open_set.push({f_score, neighbor});
      }
    }
  }

  RCLCPP_WARN(node_->get_logger(), "AStar3D: no path found after %d iterations", iterations);
  return empty_path;
}

}  // namespace pnc_nav_planners

PLUGINLIB_EXPORT_CLASS(pnc_nav_planners::AStar3D, pnc_nav_core::GlobalPlannerBase)
