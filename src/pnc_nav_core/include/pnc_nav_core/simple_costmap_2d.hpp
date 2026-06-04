// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_CORE__SIMPLE_COSTMAP_2D_HPP_
#define PNC_NAV_CORE__SIMPLE_COSTMAP_2D_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "pnc_nav_core/costmap_interface.hpp"

namespace pnc_nav_core
{

/**
 * @class SimpleCostmap2D
 * @brief Phase 1 用的轻量 2D costmap, 用固定矩形障碍验证规划闭环。
 *
 * 它不是 Nav2 Costmap2DROS 的替代品, 只提供 A-star + Pure Pursuit demo
 * 所需的最小地图查询能力。后续可替换为 Nav2 costmap、OctoMap 或
 * traversability map, 规划器接口不用改。
 */
class SimpleCostmap2D : public CostmapInterface
{
public:
  explicit SimpleCostmap2D(const rclcpp::Node::SharedPtr & node);

  uint8_t getCost(double x, double y, double z) const override;
  bool isOccupied(double x, double y, double z) const override;
  bool isInBounds(double x, double y, double z) const override;
  double getResolution() const override;
  void getBounds(
    double & min_x, double & min_y, double & min_z,
    double & max_x, double & max_y, double & max_z) const override;
  std::string getFrameId() const override;

private:
  struct ObstacleBox
  {
    double min_x{0.0};
    double min_y{0.0};
    double max_x{0.0};
    double max_y{0.0};
  };

  void loadObstacleBoxes(const std::vector<double> & values);
  double distanceToBox(double x, double y, const ObstacleBox & box) const;
  double distanceToNearestObstacle(double x, double y) const;

  std::string frame_id_{"map"};
  double origin_x_{-5.0};
  double origin_y_{-5.0};
  double width_{10.0};
  double height_{10.0};
  double resolution_{0.1};
  double lethal_radius_{0.15};
  double inflation_radius_{0.5};
  std::vector<ObstacleBox> obstacles_;
};

}  // namespace pnc_nav_core

#endif  // PNC_NAV_CORE__SIMPLE_COSTMAP_2D_HPP_
