// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_CORE__NAV2_COSTMAP_ADAPTER_HPP_
#define PNC_NAV_CORE__NAV2_COSTMAP_ADAPTER_HPP_

#include <memory>
#include <string>

#include "pnc_nav_core/costmap_interface.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace pnc_nav_core
{

/**
 * @class Nav2CostmapAdapter
 * @brief 桥接 Nav2 的 Costmap2DROS 到 pnc_nav_core 的 CostmapInterface
 *
 * 这是一个最小适配器,让你的规划器插件能复用 Nav2 成熟的 costmap。
 * Phase 1 用此快速验证;Phase 2+ 可替换为 OctoMap/ESDF 等真 3D 实现。
 */
class Nav2CostmapAdapter : public CostmapInterface
{
public:
  explicit Nav2CostmapAdapter(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  uint8_t getCost(double x, double y, double z) const override;
  bool isOccupied(double x, double y, double z) const override;
  bool isInBounds(double x, double y, double z) const override;
  double getResolution() const override;
  void getBounds(
    double & min_x, double & min_y, double & min_z,
    double & max_x, double & max_y, double & max_z) const override;
  std::string getFrameId() const override;

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
};

}  // namespace pnc_nav_core

#endif  // PNC_NAV_CORE__NAV2_COSTMAP_ADAPTER_HPP_
