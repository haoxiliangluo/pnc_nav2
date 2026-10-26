// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_CORE__TRAVERSABILITY_COSTMAP_ADAPTER_HPP_
#define PNC_NAV_CORE__TRAVERSABILITY_COSTMAP_ADAPTER_HPP_

#include <memory>
#include <string>

#include "pnc_nav_core/costmap_interface.hpp"
#include "pnc_nav_interfaces/msg/traversability_map.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pnc_nav_core
{

/**
 * @class TraversabilityCostmapAdapter
 * @brief 订阅可通行性地图消息，把它适配成规划器用的 CostmapInterface
 *
 * 为什么放在 core 而不是 pnc_nav_map：pnc_nav_map 依赖 pnc_nav_core（用 CostmapInterface），
 * 反向依赖会成环；这里只依赖消息定义，和 Nav2CostmapAdapter 同一层级。
 *
 * 语义：
 * - getCost：score<0/未知 → UNKNOWN(255)；否则 (1-score)*254
 * - isOccupied：score < lethal_score（默认 0.3）
 * - getGroundHeight / getTraversability：直接读消息数组
 */
class TraversabilityCostmapAdapter : public CostmapInterface
{
public:
  TraversabilityCostmapAdapter(
    const rclcpp::Node::SharedPtr & node,
    const std::string & topic = "traversability_map",
    double lethal_score = 0.3);

  uint8_t getCost(double x, double y, double z) const override;
  bool isOccupied(double x, double y, double z) const override;
  bool isInBounds(double x, double y, double z) const override;
  double getResolution() const override;
  void getBounds(
    double & min_x, double & min_y, double & min_z,
    double & max_x, double & max_y, double & max_z) const override;
  std::string getFrameId() const override;
  void clear() override;

  double getTraversability(double x, double y, double z) const override;
  double getGroundHeight(double x, double y) const override;

  bool hasData() const {return grid_ != nullptr;}

private:
  void msgCallback(const pnc_nav_interfaces::msg::TraversabilityMap::SharedPtr msg);
  bool worldToIndex(double x, double y, size_t & index) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<pnc_nav_interfaces::msg::TraversabilityMap>::SharedPtr sub_;
  pnc_nav_interfaces::msg::TraversabilityMap::SharedPtr grid_;
  double lethal_score_{0.3};
};

}  // namespace pnc_nav_core

#endif  // PNC_NAV_CORE__TRAVERSABILITY_COSTMAP_ADAPTER_HPP_
