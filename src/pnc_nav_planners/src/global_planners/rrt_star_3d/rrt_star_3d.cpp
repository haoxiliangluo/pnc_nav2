// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#include "pnc_nav_planners/global_planners/rrt_star_3d.hpp"

#include <cmath>
#include <algorithm>
#include <chrono>

#include "pluginlib/class_list_macros.hpp"

namespace pnc_nav_planners
{

void RRTStar3D::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const std::shared_ptr<pnc_nav_core::CostmapInterface> & costmap)
{
  node_ = node;
  name_ = name;
  costmap_ = costmap;

  node_->declare_parameter(name_ + ".max_iterations", 50000);
  node_->declare_parameter(name_ + ".step_size", 0.3);
  node_->declare_parameter(name_ + ".goal_bias", 0.1);
  node_->declare_parameter(name_ + ".rewire_radius", 0.5);
  node_->declare_parameter(name_ + ".max_planning_time", 5.0);
  node_->declare_parameter(name_ + ".goal_tolerance", 0.2);

  max_iterations_ = node_->get_parameter(name_ + ".max_iterations").as_int();
  step_size_ = node_->get_parameter(name_ + ".step_size").as_double();
  goal_bias_ = node_->get_parameter(name_ + ".goal_bias").as_double();
  rewire_radius_ = node_->get_parameter(name_ + ".rewire_radius").as_double();
  max_planning_time_ = node_->get_parameter(name_ + ".max_planning_time").as_double();
  goal_tolerance_ = node_->get_parameter(name_ + ".goal_tolerance").as_double();

  // 初始化随机数生成器
  rng_ = std::mt19937(std::random_device{}());

  RCLCPP_INFO(node_->get_logger(), "RRTStar3D configured: step=%.2f, rewire_r=%.2f",
    step_size_, rewire_radius_);
}

void RRTStar3D::activate() {}
void RRTStar3D::deactivate() {}

void RRTStar3D::cleanup()
{
  tree_.clear();
  node_.reset();
  costmap_.reset();
}

RRTNode RRTStar3D::sampleRandom() const
{
  std::uniform_real_distribution<double> dist_x(sample_min_x_, sample_max_x_);
  std::uniform_real_distribution<double> dist_y(sample_min_y_, sample_max_y_);
  std::uniform_real_distribution<double> dist_z(sample_min_z_, sample_max_z_);
  std::uniform_real_distribution<double> dist_01(0.0, 1.0);

  RRTNode sample;
  if (dist_01(rng_) < goal_bias_) {
    // 偏向目标采样
    sample = goal_node_;
  } else {
    sample.x = dist_x(rng_);
    sample.y = dist_y(rng_);
    sample.z = dist_z(rng_);
  }
  return sample;
}

int RRTStar3D::findNearest(const RRTNode & sample) const
{
  int nearest_id = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < tree_.size(); ++i) {
    double d = distance(tree_[i], sample);
    if (d < min_dist) {
      min_dist = d;
      nearest_id = static_cast<int>(i);
    }
  }
  return nearest_id;
}

RRTNode RRTStar3D::steer(const RRTNode & nearest, const RRTNode & sample) const
{
  double d = distance(nearest, sample);
  RRTNode new_node;

  if (d <= step_size_) {
    new_node = sample;
  } else {
    double ratio = step_size_ / d;
    new_node.x = nearest.x + ratio * (sample.x - nearest.x);
    new_node.y = nearest.y + ratio * (sample.y - nearest.y);
    new_node.z = nearest.z + ratio * (sample.z - nearest.z);
  }
  return new_node;
}

bool RRTStar3D::isCollisionFree(const RRTNode & from, const RRTNode & to) const
{
  if (!costmap_) return true;

  double d = distance(from, to);
  int steps = std::max(2, static_cast<int>(d / (costmap_->getResolution() * 0.5)));

  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / steps;
    double x = from.x + t * (to.x - from.x);
    double y = from.y + t * (to.y - from.y);
    double z = from.z + t * (to.z - from.z);

    if (!costmap_->isInBounds(x, y, z)) return false;
    if (costmap_->isOccupied(x, y, z)) return false;
  }
  return true;
}

std::vector<int> RRTStar3D::findNearNodes(const RRTNode & node) const
{
  std::vector<int> near_ids;
  for (size_t i = 0; i < tree_.size(); ++i) {
    if (distance(tree_[i], node) <= rewire_radius_) {
      near_ids.push_back(static_cast<int>(i));
    }
  }
  return near_ids;
}

double RRTStar3D::distance(const RRTNode & a, const RRTNode & b) const
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

nav_msgs::msg::Path RRTStar3D::extractPath(int goal_id, const std_msgs::msg::Header & header) const
{
  nav_msgs::msg::Path path;
  path.header = header;

  std::vector<int> indices;
  int current = goal_id;
  while (current >= 0) {
    indices.push_back(current);
    current = tree_[current].parent_id;
  }
  std::reverse(indices.begin(), indices.end());

  for (int idx : indices) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = tree_[idx].x;
    pose.pose.position.y = tree_[idx].y;
    pose.pose.position.z = tree_[idx].z;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path RRTStar3D::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header = start.header;

  if (!costmap_) {
    RCLCPP_ERROR(node_->get_logger(), "RRTStar3D: costmap not available");
    return empty_path;
  }

  // 设置采样范围
  costmap_->getBounds(sample_min_x_, sample_min_y_, sample_min_z_,
                      sample_max_x_, sample_max_y_, sample_max_z_);

  // 初始化树
  tree_.clear();
  RRTNode start_node;
  start_node.x = start.pose.position.x;
  start_node.y = start.pose.position.y;
  start_node.z = start.pose.position.z;
  start_node.cost = 0.0;
  start_node.parent_id = -1;
  tree_.push_back(start_node);

  // 目标
  goal_node_.x = goal.pose.position.x;
  goal_node_.y = goal.pose.position.y;
  goal_node_.z = goal.pose.position.z;

  auto start_time = std::chrono::steady_clock::now();
  int best_goal_id = -1;
  double best_goal_cost = std::numeric_limits<double>::max();

  for (int iter = 0; iter < max_iterations_; ++iter) {
    // 超时检查
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration<double>(elapsed).count() > max_planning_time_) {
      RCLCPP_WARN(node_->get_logger(), "RRTStar3D: timeout after %d iterations", iter);
      break;
    }

    // 采样
    RRTNode sample = sampleRandom();

    // 找最近节点
    int nearest_id = findNearest(sample);
    const RRTNode & nearest = tree_[nearest_id];

    // 扩展
    RRTNode new_node = steer(nearest, sample);

    // 碰撞检测
    if (!isCollisionFree(nearest, new_node)) continue;

    // RRT* rewire: 找最优父节点
    auto near_ids = findNearNodes(new_node);
    int best_parent = nearest_id;
    double best_cost = nearest.cost + distance(nearest, new_node);

    for (int nid : near_ids) {
      double candidate_cost = tree_[nid].cost + distance(tree_[nid], new_node);
      if (candidate_cost < best_cost && isCollisionFree(tree_[nid], new_node)) {
        best_parent = nid;
        best_cost = candidate_cost;
      }
    }

    new_node.parent_id = best_parent;
    new_node.cost = best_cost;
    int new_id = static_cast<int>(tree_.size());
    tree_.push_back(new_node);

    // Rewire 邻居
    for (int nid : near_ids) {
      double rewire_cost = new_node.cost + distance(new_node, tree_[nid]);
      if (rewire_cost < tree_[nid].cost && isCollisionFree(new_node, tree_[nid])) {
        tree_[nid].parent_id = new_id;
        tree_[nid].cost = rewire_cost;
      }
    }

    // 检查是否到达目标
    if (distance(new_node, goal_node_) < goal_tolerance_) {
      if (new_node.cost < best_goal_cost) {
        best_goal_id = new_id;
        best_goal_cost = new_node.cost;
      }
    }
  }

  if (best_goal_id >= 0) {
    RCLCPP_INFO(node_->get_logger(), "RRTStar3D: path found, cost=%.2f, tree_size=%zu",
      best_goal_cost, tree_.size());
    return extractPath(best_goal_id, start.header);
  }

  RCLCPP_WARN(node_->get_logger(), "RRTStar3D: no path found, tree_size=%zu", tree_.size());
  return empty_path;
}

}  // namespace pnc_nav_planners

PLUGINLIB_EXPORT_CLASS(pnc_nav_planners::RRTStar3D, pnc_nav_core::GlobalPlannerBase)
