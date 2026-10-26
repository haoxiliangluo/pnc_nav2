// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_MAP__ELEVATION_MAP_HPP_
#define PNC_NAV_MAP__ELEVATION_MAP_HPP_

#include <cstdint>
#include <limits>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pnc_nav_interfaces/msg/traversability_map.hpp"

#include "pnc_nav_map/traversability.hpp"

namespace pnc_nav_map
{

/**
 * @class ElevationMap
 * @brief 2.5D 高程图：每格累积点云的 z 统计量（mean/min/max/count）
 *
 * 定位：Phase 3 的"地图"层。它实现的是高程数据 + 可通行性评估，
 * 通过 ElevationCostmap（下一步）适配成规划器用的 CostmapInterface。
 *
 * 约定：坐标系为 map（z 向上，地面 z≈0）；分辨率与静态图一致时最省心（默认 0.1）。
 */
class ElevationMap
{
public:
  ElevationMap(double resolution, double origin_x, double origin_y, int width, int height);

  /// 清空所有格子（保留尺寸）
  void reset();
  /// 重置尺寸/原点（滚动窗口或换图时用）
  void setBounds(double origin_x, double origin_y, int width, int height);

  /// 写入一个点（map 系）；超出范围返回 false
  bool addPoint(double x, double y, double z);

  bool worldToCell(double x, double y, int & mx, int & my) const;
  bool inBounds(double x, double y) const;

  /// 该点地面高度（格内 z 均值）；无数据返回 false
  bool getGroundHeight(double x, double y, double & z) const;
  /// 指定格是否有数据
  bool hasData(int mx, int my) const;

  /// 该点可通行性（8 邻域 + 中心差分坡度）
  Traversability traversabilityAt(
    double x, double y, const TraversabilityParams & params) const;

  /// 可视化：代价 0-100（未知 -1）
  nav_msgs::msg::OccupancyGrid toOccupancyGrid(const TraversabilityParams & params) const;
  /// 完整地形信息（给 RViz / 评估脚本）
  pnc_nav_interfaces::msg::TraversabilityMap toTraversabilityMap(
    const TraversabilityParams & params) const;

  double resolution() const {return resolution_;}
  double originX() const {return origin_x_;}
  double originY() const {return origin_y_;}
  int width() const {return width_;}
  int height() const {return height_;}

private:
  struct Cell
  {
    double z_sum{0.0};
    double z_min{std::numeric_limits<double>::infinity()};
    double z_max{-std::numeric_limits<double>::infinity()};
    uint32_t count{0};

    bool hasData() const {return count > 0;}
    double mean() const {return count > 0 ? z_sum / count : std::numeric_limits<double>::quiet_NaN();}
  };

  int indexOf(int mx, int my) const {return my * width_ + mx;}
  bool cellMean(int mx, int my, double & z) const;

  double resolution_{0.1};
  double origin_x_{0.0};
  double origin_y_{0.0};
  int width_{0};
  int height_{0};
  std::vector<Cell> cells_;
};

}  // namespace pnc_nav_map

#endif  // PNC_NAV_MAP__ELEVATION_MAP_HPP_
