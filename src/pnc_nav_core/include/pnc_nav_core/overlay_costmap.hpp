// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_CORE__OVERLAY_COSTMAP_HPP_
#define PNC_NAV_CORE__OVERLAY_COSTMAP_HPP_

#include <memory>
#include <string>

#include "pnc_nav_core/costmap_interface.hpp"

namespace pnc_nav_core
{

/**
 * @class OverlayCostmap
 * @brief 给全局规划器用的"静态整图 + 局部滚动窗口"叠加视图
 *
 * - 范围/分辨率/坐标系：跟随静态层（全局规划器能看到整张地图）
 * - 查询：落在局部窗口内就用局部层（含激光障碍），否则用静态层
 *
 * 作用：A* 在重规划时能"看见"局部窗口里的动态障碍，从而绕过它；
 * 而不会像纯局部层那样只能搜 8m。
 */
class OverlayCostmap : public CostmapInterface
{
public:
  OverlayCostmap(
    std::shared_ptr<CostmapInterface> base,
    std::shared_ptr<CostmapInterface> overlay)
  : base_(std::move(base)), overlay_(std::move(overlay)) {}

  uint8_t getCost(double x, double y, double z) const override
  {
    // 窗口内如果有数据就用窗口（含动态/地形障碍），否则回退静态层：
    // 这样高程图/点云还没收到数据时不会让全局规划"看不见地图"。
    if (overlay_ && overlay_->isInBounds(x, y, z)) {
      const uint8_t cost = overlay_->getCost(x, y, z);
      if (cost != cost_values::UNKNOWN) {
        return cost;
      }
    }
    return base_ ? base_->getCost(x, y, z) : cost_values::UNKNOWN;
  }

  bool isOccupied(double x, double y, double z) const override
  {
    if (overlay_ && overlay_->isInBounds(x, y, z) &&
      overlay_->getCost(x, y, z) != cost_values::UNKNOWN)
    {
      return overlay_->isOccupied(x, y, z);
    }
    return base_ ? base_->isOccupied(x, y, z) : true;
  }

  bool isInBounds(double x, double y, double z) const override
  {
    return base_ ? base_->isInBounds(x, y, z) : false;
  }

  double getResolution() const override
  {
    return base_ ? base_->getResolution() : 0.05;
  }

  void getBounds(
    double & min_x, double & min_y, double & min_z,
    double & max_x, double & max_y, double & max_z) const override
  {
    if (base_) {
      base_->getBounds(min_x, min_y, min_z, max_x, max_y, max_z);
    } else {
      min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
    }
  }

  std::string getFrameId() const override
  {
    return base_ ? base_->getFrameId() : "map";
  }

  void clear() override
  {
    if (overlay_) {
      overlay_->clear();
    }
  }

private:
  std::shared_ptr<CostmapInterface> base_;
  std::shared_ptr<CostmapInterface> overlay_;
};

}  // namespace pnc_nav_core

#endif  // PNC_NAV_CORE__OVERLAY_COSTMAP_HPP_
