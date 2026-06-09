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
std::vector<GridIndex> AStar2D::getNeighbors(const GridIndex & current) const
{
    std::vector<GridIndex> neighbors;
    // 8-邻域
    const int dirs[][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    for(const auto & dir :dirs)
    {
        GridIndex n{current.x + dir[0], current.y + dir[1]};
        neighbors.push_back(n);
    }
    return neighbors;
}

  // 回溯路径
nav_msgs::msg::Path AStar2D::reconstructPath(
  const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
  const GridIndex & start,
  const GridIndex & goal,
  const std_msgs::msg::Header & header) const
{
  nav_msgs::msg::Path path;
  path.header = header;
  GridIndex current = goal;
  std::vector<GridIndex> index_path;
  while (!(current == start)) 
  {
    index_path.push_back(current);
    auto it = came_from.find(current);
    if (it == came_from.end()) break;
    current = it->second;
  }
  index_path.push_back(start);
  std::reverse(index_path.begin(), index_path.end());
  for(const auto & idx : index_path)
  {
    geometry_msgs::msg::PoseStamped pose;
    gridToWorld(idx,pose.pose.position.x,pose.pose.position.y);
    pose.header = header;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0; // 无旋转
    path.poses.push_back(pose);
  }


  return smoothPath(path);
}

nav_msgs::msg::Path AStar2D::smoothPath(const nav_msgs::msg::Path & raw_path) const
{
  return raw_path;
}

nav_msgs::msg::Path AStar2D::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
    nav_msgs::msg::Path empty_path;
    empty_path.header = start.header;
    if(costmap_ == nullptr)
    {
      RCLCPP_ERROR(node_->get_logger(), "Costmap is not set");
      return empty_path;
    }
// 3. start/goal worldToGrid
    GridIndex start_idx = worldToGrid(start.pose.position.x, start.pose.position.y);
    GridIndex goal_idx = worldToGrid(goal.pose.position.x, goal.pose.position.y);
// 4. 检查 goal和start是否在障碍里
    if (!costmap_->isInBounds(start.pose.position.x, start.pose.position.y, start.pose.position.z)) {
      RCLCPP_WARN(node_->get_logger(), "AStar2D: start out of bounds");
      return empty_path;
    }
    if(costmap_->isOccupied(start.pose.position.x, start.pose.position.y, 0.0))
    {
      RCLCPP_WARN(node_->get_logger(), "Start is occupied");
      return empty_path;
    }
    if (!costmap_->isInBounds(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)) {
    RCLCPP_WARN(node_->get_logger(), "AStar2D: goal out of bounds");
    return empty_path;
    }
    if(costmap_->isOccupied(goal.pose.position.x, goal.pose.position.y, 0.0))
    {
      RCLCPP_WARN(node_->get_logger(), "Goal is occupied");
      return empty_path;
    }

// 5. 初始化 open_set / g_score / came_from / closed_set
    using PQElement = std::pair<double, GridIndex>;
    std::unordered_map<GridIndex,double,GridIndexHash> g_score;
    std::unordered_map<GridIndex,GridIndex,GridIndexHash> came_from;
    std::unordered_map<GridIndex,bool,GridIndexHash> closed_set;
    auto cmp = [](const PQElement & a,const PQElement & b){return a.first > b.first;};
    std::priority_queue<PQElement,std::vector<PQElement>,decltype(cmp)> open_set(cmp);
    g_score[start_idx] = 0.0;
    open_set.push({heuristic_weight_*heuristic(start_idx, goal_idx), start_idx});

// 6. while open_set 不空
    int iterations = 0;
    while(!open_set.empty() && iterations <= max_iterations_)
    {
      iterations++;
      auto current = open_set.top().second;
      open_set.pop();
      
      if(current == goal_idx)
      {
        return reconstructPath(came_from, start_idx, goal_idx, start.header);
      }
      if(closed_set[current])continue;
      closed_set[current] = true;
      for(const auto & neighbor :getNeighbors(current))
      {
        if(closed_set[neighbor])continue;
        closed_set[neighbor] = true;
        double wx,wy;
        gridToWorld(neighbor,wx,wy);

        if(!costmap_->isInBounds(wx, wy, 0.0))continue;
        if(costmap_->isOccupied(wx, wy, 0.0))continue;
        if (!allow_unknown_ && costmap_->getCost(wx, wy, 0.0) == pnc_nav_core::cost_values::UNKNOWN) {
        continue;
        
        double dx = std::abs(neighbor.x - current.x);
        double dy = std::abs(neighbor.y - current.y);
        double move_cost = std::sqrt(dx * dx + dy * dy);// 计算移动代价

        uint8_t cell_cost = costmap_->getCost(wx, wy, 0.0);
        double cost_factor = 1.0 + static_cast<double>(cell_cost) / 252.0;

        double tentative_g = g_score[current] + move_cost * cost_factor;// 计算新的g值
        if(!g_score.count(neighbor) || tentative_g < g_score[neighbor])
        {
          g_score[neighbor] = tentative_g;
          came_from[neighbor] = current;
          double f = tentative_g + heuristic_weight_ * heuristic(neighbor, goal_idx);// 计算f值
          open_set.push({f, neighbor});
        }
          
        }
      }

    }
    RCLCPP_WARN(node_->get_logger(), "AStar2D: no path found after %d iterations", iterations);
    return empty_path;

}// namespace pnc_nav_planners
}
PLUGINLIB_EXPORT_CLASS(pnc_nav_planners::AStar2D, pnc_nav_core::GlobalPlannerBase)
