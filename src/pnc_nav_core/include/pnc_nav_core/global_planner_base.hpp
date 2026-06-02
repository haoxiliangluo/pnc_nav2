// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_CORE__GLOBAL_PLANNER_BASE_HPP_
#define PNC_NAV_CORE__GLOBAL_PLANNER_BASE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pnc_nav_core/costmap_interface.hpp"

namespace pnc_nav_core
{

/**
 * @class GlobalPlannerBase
 * @brief 全局规划器插件基类
 *
 * 所有全局规划算法（A*, RRT*, PCT等）需继承此基类并实现纯虚函数。
 * 通过 pluginlib 注册后可在运行时动态加载和切换。
 */
class GlobalPlannerBase
{
public:
  using SharedPtr = std::shared_ptr<GlobalPlannerBase>;

  virtual ~GlobalPlannerBase() = default;

  /**
   * @brief 配置规划器
   * @param node ROS节点指针，用于获取参数和创建发布者
   * @param name 规划器实例名称（用于参数命名空间）
   * @param costmap 代价地图接口
   */
  virtual void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name,
    const std::shared_ptr<CostmapInterface> & costmap) = 0;

  /**
   * @brief 激活规划器（生命周期管理）
   */
  virtual void activate() {}

  /**
   * @brief 停用规划器（生命周期管理）
   */
  virtual void deactivate() {}

  /**
   * @brief 清理资源
   */
  virtual void cleanup() = 0;

  /**
   * @brief 计算全局路径
   * @param start 起始位姿
   * @param goal 目标位姿
   * @return 规划的路径，空路径表示规划失败
   */
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;

  /**
   * @brief 获取规划器名称
   */
  virtual std::string getName() const = 0;
};

}  // namespace pnc_nav_core

#endif  // PNC_NAV_CORE__GLOBAL_PLANNER_BASE_HPP_
