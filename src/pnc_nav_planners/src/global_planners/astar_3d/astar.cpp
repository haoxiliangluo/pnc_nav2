// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_planners/global_planners/astar.hpp"

#include <cmath>
#include <algorithm>

#include "pluginlib/class_list_macros.hpp"

namespace pnc_nav_planners
{


std::string AStar2D::getName() const
{
  return "AStar2D";
}

void AStar2D::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const std::shared_ptr<pnc_nav_core::CostmapInterface> & costmap)
{
    node_ = node;
    name_ = name;
    costmap_ = costmap;
    // 从参数服务器获取配置

    node_->declare_parameter(name_ + ".resolution", 0.05);
    node_->declare_parameter(name_ + ".heuristic_weight", 1.2);
    node_->declare_parameter(name_ + ".max_iterations", 100000);
    node_->declare_parameter(name_ + ".allow_unknown", false);
    node_->declare_parameter(name_ + ".diagonal_movement", true);

    resolution_ = node_->get_parameter(name_ + ".resolution").as_double();
    heuristic_weight_ = node_->get_parameter(name_ + ".heuristic_weight").as_double();
    max_iterations_ = node_->get_parameter(name_ + ".max_iterations").as_int();
    allow_unknown_ = node_->get_parameter(name_ + ".allow_unknown").as_bool();
    diagonal_movement_ = node_->get_parameter(name_ + ".diagonal_movement").as_bool();
}

void AStar2D::activate()
{
  RCLCPP_INFO(node_->get_logger(), "AStar2D activated");
}

void AStar2D::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "AStar2D deactivated");
}

void AStar2D::cleanup()
{
  node_.reset();
  costmap_.reset();
  RCLCPP_INFO(node_->get_logger(), "AStar2D cleaned up");
}

GridIndex AStar2D::worldToGrid(double x, double y) const
{
  GridIndex idx;
  idx.x = static_cast<int>(std::floor(x / resolution_));
  idx.y = static_cast<int>(std::floor(y / resolution_));

  return idx;
}

void AStar2D::gridToWorld(const GridIndex & idx, double & x, double & y) const
{
  x = (idx.x + 0.5) * resolution_;
  y = (idx.y + 0.5) * resolution_;
}
double AStar2D::heuristic(const GridIndex & a, const GridIndex & b) const
{
  // 使用欧几里得距离作为启发式函数（8邻域）
  return std::sqrt((a.x -b.x) * (a.x -b.x) + (a.y -b.y) * (a.y -b.y)) * resolution_;
  // 也可以使用曼哈顿距离（四邻域）： 
  //  return (std::abs(a.x - b.x) + std::abs(a.y - b.y)) * resolution_;
  //  octile距离（八邻域）： 
  //  return std::max(std::abs(a.x -b.x),std::abs(a.y - b.y))*resolution_ +std::min(std::abs(a.x -b.x),std::abs(a.y - b.y))*resolution_*(std::sqrt(2)-1);
}  

// 获取邻居节点
std::vector<GridIndex> getNeighbors(const GridIndex & current)
{
    std::vector<GridIndex> neighbors;
    // 8-邻域
    const int dirs[][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    for(const auto & dir :dirs)
    {
        GridIndex n{current.x + dir[0], current.y + dir[1], 0};
        neighbors.push_back(n);
    }
    return neighbors;
}

  // 回溯路径
nav_msgs::msg::Path reconstructPath(
const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
const GridIndex & start,
const GridIndex & goal,
const std_msgs::msg::Header & header)
{

}

// 路径平滑
nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path & raw_path)
{

}

nav_msgs::msg::Path AStar2D::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
    nav_msgs::msg::Path empty_path;
    empty_path.header = start.header;
// 1. 创建 empty_path
// 2. 检查 costmap_
// 3. start/goal worldToGrid
// 4. 检查 goal 是否在障碍里
// 5. 初始化 open_set / g_score / came_from / closed_set
// 6. while open_set 不空
// 7. 取 f 最小的 current
// 8. 如果 current == goal，reconstructPath 返回
// 9. 遍历邻居
// 10. gridToWorld 后查 costmap
// 11. 计算 tentative_g
// 12. 更新 g_score / came_from / open_set
// 13. 超过 max_iterations 返回空 path

    return empty_path;
}


}// namespace pnc_nav_planners

PLUGINLIB_EXPORT_CLASS(pnc_nav_planners::AStar2D, pnc_nav_core::GlobalPlannerBase)
