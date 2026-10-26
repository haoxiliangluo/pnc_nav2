// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_MAP__ELEVATION_BUILDER_HPP_
#define PNC_NAV_MAP__ELEVATION_BUILDER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"

#include "pnc_nav_map/elevation_map.hpp"
#include "pnc_nav_map/traversability.hpp"

namespace pnc_nav_map
{

/// 高程图构建参数
struct ElevationBuilderParams
{
  std::string cloud_topic{"points"};
  std::string frame_id{"map"};
  std::string robot_frame{"base_link"};
  double window_size{8.0};       // 滚动窗口边长 (m)
  double resolution{0.1};        // 高程格分辨率 (m)
  int stride{8};                 // 每 N 个点取一个
  double min_range{0.3};         // 传感器系水平距离下限 (m)
  double max_range{8.0};         // 传感器系水平距离上限 (m)
  double min_z{-0.30};           // map 系高度带下限（**包含地面**，地形需要）
  double max_z{1.50};            // map 系高度带上限
  double recenter_threshold{1.0};// 机器人移动超过它才滚动窗口 (m)
  double fill_radius{0.4};       // 未知格填充半径 (m)，用于补 16 线激光的地面空洞
  double robot_clear_radius{0.45};// 机器人自身半径内的点直接丢弃（否则会把自己当障碍）
  TraversabilityParams trav;     // 可通行性阈值
};

/**
 * @class ElevationBuilder
 * @brief /points → ElevationMap（2.5D 高程图）
 *
 * - 滚动窗口：以机器人为中心，移动超过 recenter_threshold 时重建（旧数据丢弃，
 *   由激光在 ~1s 内重新填满）；这是 Phase 3 第一版的简化，后续可做增量更新
 * - 线程模型：与 NavServer 一致，假设单线程执行器（cloud 回调与 update 串行）
 */
class ElevationBuilder
{
public:
  ElevationBuilder(
    const rclcpp::Node::SharedPtr & node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const ElevationBuilderParams & params);

  /// 每周期调用：按机器人位置滚动窗口
  void update(const geometry_msgs::msg::PoseStamped & robot_pose);

  std::shared_ptr<ElevationMap> map() const {return map_;}
  const ElevationBuilderParams & params() const {return params_;}

  /// 可视化导出
  nav_msgs::msg::OccupancyGrid occupancyGrid() const;
  pnc_nav_interfaces::msg::TraversabilityMap traversabilityMap() const;

  size_t pointCount() const {return point_count_;}

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void recenter(double x, double y);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  ElevationBuilderParams params_;
  std::shared_ptr<ElevationMap> map_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  bool has_center_{false};
  double center_x_{0.0};
  double center_y_{0.0};
  size_t point_count_{0};
};

}  // namespace pnc_nav_map

#endif  // PNC_NAV_MAP__ELEVATION_BUILDER_HPP_
