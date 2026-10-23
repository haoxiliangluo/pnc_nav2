// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License
//
// 教学示例插件：StraightLinePlanner
//
// 它故意不实现任何搜索算法——只把起点到目标连成直线并等距采样。
// 目的是让读者看清"一个全局规划器插件最少要实现什么"：
//   1. 继承 pnc_nav_core::GlobalPlannerBase
//   2. 实现 configure / createPlan / cleanup / getName
//   3. 用 PLUGINLIB_EXPORT_CLASS 导出，并在 plugins.xml 注册

#ifndef PNC_NAV_EXAMPLES__STRAIGHT_LINE_PLANNER_HPP_
#define PNC_NAV_EXAMPLES__STRAIGHT_LINE_PLANNER_HPP_

#include <memory>
#include <string>

#include "pnc_nav_core/global_planner_base.hpp"

namespace pnc_nav_examples
{

class StraightLinePlanner : public pnc_nav_core::GlobalPlannerBase
{
public:
  void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name,
    const std::shared_ptr<pnc_nav_core::CostmapInterface> & costmap) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  void cleanup() override;

  std::string getName() const override {return "StraightLinePlanner";}

private:
  rclcpp::Node::SharedPtr node_;
  std::string name_;
  std::shared_ptr<pnc_nav_core::CostmapInterface> costmap_;

  // 参数（命名空间为 <插件实例名>.*，例如 global_planner.step）
  double step_{0.1};        // 采样间距 (m)
  double max_length_{50.0};  // 超过这个长度直接返回空路径（演示失败路径）
};

}  // namespace pnc_nav_examples

#endif  // PNC_NAV_EXAMPLES__STRAIGHT_LINE_PLANNER_HPP_
