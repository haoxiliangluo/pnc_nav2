// Copyright (c) 2024, PNC Nav2
// Licensed under MIT License

#ifndef PNC_NAV_MAP__TRAVERSABILITY_HPP_
#define PNC_NAV_MAP__TRAVERSABILITY_HPP_

#include <cstdint>
#include <vector>

namespace pnc_nav_map
{

/// 可通行性参数（第一版：可解释优先）
struct TraversabilityParams
{
  double slope_max_deg{20.0};      // 坡度超过它 → 分项 0
  double step_max_m{0.10};         // 邻域高差超过它 → 分项 0
  double roughness_max_m{0.05};    // 粗糙度超过它 → 分项 0
};

/// 单格评估结果
struct Traversability
{
  double slope_rad{0.0};
  double step_m{0.0};
  double roughness_m{0.0};
  double score{-1.0};   // [0,1]；-1 = 无数据
};

/// 8 邻域顺序（EvaluateTraversability 的入参约定）
enum NeighborIndex : int
{
  kEast = 0, kNorthEast = 1, kNorth = 2, kNorthWest = 3,
  kWest = 4, kSouthWest = 5, kSouth = 6, kSouthEast = 7
};

/// 由高程梯度算坡度角（弧度）
double slopeRadFromGradient(double dz_dx, double dz_dy);

/// 邻域最大高差（忽略 NaN）
double stepFromNeighbors(double center_z, const std::vector<double> & neighbor_zs);

/// 高程样本标准差（样本 < 2 返回 0）
double roughnessFromSamples(const std::vector<double> & zs);

/**
 * @brief 综合可通行性
 * @param center_z 中心格高程
 * @param neighbors 8 邻域高程（顺序见 NeighborIndex；NaN 表示无数据）
 * @param cell_size 格距 (m)
 * @param params 阈值
 * @return 各分项 + score（score = min(三分项)，无数据时 -1）
 */
Traversability evaluateTraversability(
  double center_z,
  const std::vector<double> & neighbors,
  double cell_size,
  const TraversabilityParams & params);

/// 可通行性 → 代价（score<0 返回 255=未知；1→0，0→lethal_cost）
uint8_t traversabilityToCost(double score, uint8_t lethal_cost = 254);

}  // namespace pnc_nav_map

#endif  // PNC_NAV_MAP__TRAVERSABILITY_HPP_
