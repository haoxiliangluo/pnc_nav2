// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_CORE__ROLLING_COSTMAP_HPP_
#define PNC_NAV_CORE__ROLLING_COSTMAP_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"

#include "pnc_nav_core/costmap_interface.hpp"
#include "pnc_nav_core/nav2_costmap_adapter.hpp"

namespace pnc_nav_core
{

/**
 * @class RollingCostmap
 * @brief 滚动窗口 + 障碍层代价地图（给局部规划器用）
 *
 * 与 Nav2 的对应关系：
 * - 相当于 Nav2 的 local_costmap（滚动窗口）+ obstacle_layer + inflation_layer 的精简版
 * - 没有：voxel 层、raytrace 清除、多图层插件体系、多机器人命名空间
 *
 * 工作方式：
 * 1. 订阅 LaserScan，把每束命中点按 TF 投到 map 系，存成带时间戳的障碍点（TTL 自动过期）
 * 2. 每周期 update(robot_pose)：过期旧点；机器人移动超过阈值或有新扫描时重建窗口网格
 * 3. 重建时：静态图障碍（可选）+ 激光障碍 → 一次性膨胀，查询变成 O(1) 查表
 *
 * 线程模型：与 NavServer 一样假设单线程执行器（扫描回调与 update 串行），因此不加锁。
 */
class RollingCostmap : public CostmapInterface
{
public:
  RollingCostmap(
    const rclcpp::Node::SharedPtr & node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<Nav2CostmapAdapter> static_layer);

  // --- CostmapInterface ---
  uint8_t getCost(double x, double y, double z) const override;
  bool isOccupied(double x, double y, double z) const override;
  bool isInBounds(double x, double y, double z) const override;
  double getResolution() const override {return resolution_;}
  void getBounds(
    double & min_x, double & min_y, double & min_z,
    double & max_x, double & max_y, double & max_z) const override;
  std::string getFrameId() const override {return frame_id_;}
  void clear() override;

  /// 每周期调用：过期旧障碍点，必要时重建网格
  void update(const geometry_msgs::msg::PoseStamped & robot_pose);

  /// 生成可视化栅格（obstacles_only=true 时只含激光障碍，不含静态图）
  nav_msgs::msg::OccupancyGrid toOccupancyGrid(bool obstacles_only) const;

  size_t obstaclePointCount() const {return points_.size();}
  bool enabled() const {return enabled_;}

private:
  struct ObstaclePoint
  {
    double x;   // map 系
    double y;
    std::chrono::steady_clock::time_point stamp;
  };

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void expirePoints();
  void rebuildGrid(const geometry_msgs::msg::PoseStamped & robot_pose);
  bool worldToCell(double wx, double wy, int & mx, int & my) const;
  bool worldToCellPrecomputed(double wx, double wy, int & mx, int & my) const;
  void inflateMask(const std::vector<uint8_t> & mask, std::vector<uint8_t> & out) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<Nav2CostmapAdapter> static_layer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // --- 参数 ---
  bool enabled_{true};
  double window_size_{8.0};       // 窗口边长 (m)
  double resolution_{0.05};       // 窗口分辨率 (m/cell)
  double obstacle_ttl_{3.0};      // 激光障碍点存活时间 (s)
  double max_range_{6.0};         // 只使用这个范围内的激光 (m)
  double inflation_radius_{0.35}; // 膨胀半径 (m)
  double robot_radius_{0.25};     // 内切半径 (m)
  double cost_scaling_factor_{3.0};
  bool include_static_{true};     // 是否把静态图障碍叠加到窗口里
  bool unknown_as_occupied_{false};

  // --- 窗口状态 ---
  std::string frame_id_{"map"};
  double origin_x_{0.0};
  double origin_y_{0.0};
  int width_{0};
  int height_{0};
  std::vector<uint8_t> merged_cost_;    // 静态 + 激光 + 膨胀（局部规划器查询用）
  std::vector<uint8_t> obstacle_cost_;  // 仅激光 + 膨胀（可视化/调试用）
  bool dirty_{true};
  bool has_center_{false};
  double last_center_x_{0.0};
  double last_center_y_{0.0};

  std::deque<ObstaclePoint> points_;
};

}  // namespace pnc_nav_core

#endif  // PNC_NAV_CORE__ROLLING_COSTMAP_HPP_
