#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>

#include "pnc_nav_core/global_planner_base.hpp"

namespace pnc_nav_planners
{
struct GridIndex
{
    int x=0;
    int y=0;

    bool operator==(const GridIndex & other) const
    {
        return x == other.x && y == other.y ;
    }
};
struct GridIndexHash
{
    size_t operator()(const GridIndex & idx) const
    {
        size_t h1 = std::hash<int>()(idx.x);
        size_t h2 = std::hash<int>()(idx.y);
        return h1 ^ (h2 << 16);
    }
};
class AStar2D : public pnc_nav_core::GlobalPlannerBase
{
 public:
    AStar2D() = default;
   ~AStar2D() override = default;

   void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name,
    const std::shared_ptr<pnc_nav_core::CostmapInterface> & costmap) override ;

   void activate() override ;
   void deactivate() override ;
   void cleanup() override ;

   nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override ;

   std::string getName() const override ;
   private:
   // 世界坐标 → 栅格索引
  GridIndex worldToGrid(double x, double y) const;
  // 栅格索引 → 世界坐标
  void gridToWorld(const GridIndex & idx, double & x, double & y) const;


  // 获取邻居节点
  std::vector<GridIndex> getNeighbors(const GridIndex & current) const;

  // 回溯路径
  nav_msgs::msg::Path reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    const GridIndex & start,
    const GridIndex & goal,
    const std_msgs::msg::Header & header) const;

  // 路径平滑
  nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path & raw_path) const;


  // --- 成员变量 ---
  rclcpp::Node::SharedPtr node_;// ROS节点指针
  std::string name_; // 规划器实例名称
  std::shared_ptr<pnc_nav_core::CostmapInterface> costmap_;// 代价地图接口

  // 启发式函数
  double heuristic(const GridIndex & a, const GridIndex & b) const;
   // 参数
  double resolution_{0.05};// 栅格分辨率 (m)
  double heuristic_weight_{1.2};// 启发式权重
  int max_iterations_{100000};// 最大迭代次数
  bool allow_unknown_{false};// 是否允许未知区域
  bool diagonal_movement_{true};// 是否允许对角移动

  // 地图边界（栅格坐标）
  int grid_min_x_{0}, grid_max_x_{0}; 
  int grid_min_y_{0}, grid_max_y_{0};

};

}