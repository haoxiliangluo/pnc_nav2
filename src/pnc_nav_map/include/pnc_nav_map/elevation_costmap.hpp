// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_MAP__ELEVATION_COSTMAP_HPP_
#define PNC_NAV_MAP__ELEVATION_COSTMAP_HPP_

#include <memory>
#include <string>

#include "pnc_nav_core/costmap_interface.hpp"
#include "pnc_nav_map/elevation_map.hpp"
#include "pnc_nav_map/traversability.hpp"

namespace pnc_nav_map
{

/**
 * @class ElevationCostmap
 * @brief 把 2.5D 高程图的"可通行性"适配成规划器用的 CostmapInterface
 *
 * 语义（第一版）：
 * - getCost：score<0（未知）→ UNKNOWN(255)；否则 (1-score)*254
 * - isOccupied：score < lethal_score（默认 0.3）→ true；未知不当作障碍
 * - getTraversability / getGroundHeight：直接透传高程图
 * - getDistanceToObstacle：未实现（-1），留给距离场版本
 */
class ElevationCostmap : public pnc_nav_core::CostmapInterface
{
public:
  explicit ElevationCostmap(
    std::shared_ptr<ElevationMap> map,
    TraversabilityParams params = TraversabilityParams{},
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

private:
  std::shared_ptr<ElevationMap> map_;
  TraversabilityParams params_;
  double lethal_score_{0.3};
};

}  // namespace pnc_nav_map

#endif  // PNC_NAV_MAP__ELEVATION_COSTMAP_HPP_
