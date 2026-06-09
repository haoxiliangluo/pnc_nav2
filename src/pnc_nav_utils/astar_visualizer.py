#!/usr/bin/env python3
"""
A* 路径规划可视化工具

功能：
  - 读取 nav_params.yaml 配置（代价地图 + 规划器参数）
  - 用 Python 实现正确的 A* 算法（修复 C++ 版本中的 bug）
  - 左键点击：设置 start 点（绿色三角）
  - 右键点击：设置 goal 点（红色星）
  - 设置完 start+goal 后自动运行 A* 并显示路径
  - a 键：切换是否显示 A* 搜索过程动画
  - c 键：清除路径和 start/goal
  - q 键：退出

用法：
  python3 astar_visualizer.py
  python3 astar_visualizer.py --config src/pnc_nav_bringup/config/nav_params.yaml
"""

import argparse
import heapq
import math
import sys
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import yaml
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch


# ──────────────────────────────────────────────
# 数据结构
# ──────────────────────────────────────────────

@dataclass
class CostmapConfig:
    origin_x: float = -5.0
    origin_y: float = -5.0
    width: float = 10.0
    height: float = 10.0
    resolution: float = 0.1
    lethal_radius: float = 0.25
    inflation_radius: float = 0.5
    obstacles: List[Tuple[float, float, float, float]] = field(default_factory=list)


@dataclass
class PlannerConfig:
    resolution: float = 0.1
    heuristic_weight: float = 1.2
    max_iterations: int = 100000
    allow_unknown: bool = False
    diagonal_movement: bool = True


@dataclass
class GridIndex:
    x: int = 0
    y: int = 0

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)


@dataclass
class SearchResult:
    path: List[Tuple[float, float]] = field(default_factory=list)
    path_length: float = 0.0
    path_cost: float = 0.0
    iterations: int = 0
    elapsed_ms: float = 0.0
    success: bool = False
    open_set: List[GridIndex] = field(default_factory=list)
    closed_set: List[GridIndex] = field(default_factory=list)


# ──────────────────────────────────────────────
# 代价地图（复现 SimpleCostmap2D 逻辑）
# ──────────────────────────────────────────────

class SimpleCostmap2D:
    def __init__(self, config: CostmapConfig):
        self.cfg = config
        self.nx = int(round(config.width / config.resolution))
        self.ny = int(round(config.height / config.resolution))
        self._build_cost_grid()

    def _distance_to_box(self, x, y, box):
        dx = max(box[0] - x, 0.0, x - box[2])
        dy = max(box[1] - y, 0.0, y - box[3])
        return math.hypot(dx, dy)

    def _build_cost_grid(self):
        cfg = self.cfg
        self.cost_grid = np.zeros((self.ny, self.nx), dtype=np.float64)

        for iy in range(self.ny):
            for ix in range(self.nx):
                x = cfg.origin_x + (ix + 0.5) * cfg.resolution
                y = cfg.origin_y + (iy + 0.5) * cfg.resolution

                if not cfg.obstacles:
                    self.cost_grid[iy, ix] = 0.0
                    continue

                dist = min(self._distance_to_box(x, y, b) for b in cfg.obstacles)
                if dist <= cfg.lethal_radius:
                    self.cost_grid[iy, ix] = 254.0  # LETHAL
                elif dist <= cfg.inflation_radius:
                    ratio = 1.0 - (dist - cfg.lethal_radius) / max(1e-6, cfg.inflation_radius - cfg.lethal_radius)
                    self.cost_grid[iy, ix] = np.clip(180.0 * ratio, 1.0, 180.0)
                else:
                    self.cost_grid[iy, ix] = 0.0  # FREE_SPACE

    def world_to_grid(self, x, y):
        gx = int(math.floor(x / self.cfg.resolution))
        gy = int(math.floor(y / self.cfg.resolution))
        return GridIndex(gx, gy)

    def grid_to_world(self, idx):
        wx = (idx.x + 0.5) * self.cfg.resolution
        wy = (idx.y + 0.5) * self.cfg.resolution
        return wx, wy

    def is_in_bounds(self, x, y):
        return (self.cfg.origin_x <= x <= self.cfg.origin_x + self.cfg.width and
                self.cfg.origin_y <= y <= self.cfg.origin_y + self.cfg.height)

    def get_cost(self, x, y):
        if not self.is_in_bounds(x, y):
            return 255  # UNKNOWN
        if not self.cfg.obstacles:
            return 0
        dist = min(self._distance_to_box(x, y, b) for b in self.cfg.obstacles)
        if dist <= self.cfg.lethal_radius:
            return 254  # LETHAL
        if dist <= self.cfg.inflation_radius:
            ratio = 1.0 - (dist - self.cfg.lethal_radius) / max(1e-6, self.cfg.inflation_radius - self.cfg.lethal_radius)
            return int(np.clip(180.0 * ratio, 1.0, 180.0))
        return 0  # FREE_SPACE

    def is_occupied(self, x, y):
        return self.get_cost(x, y) >= 254  # >= LETHAL

    def get_cost_at_grid(self, ix, iy):
        """直接从预计算的网格获取代价"""
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            return int(self.cost_grid[iy, ix])
        return 255


# ──────────────────────────────────────────────
# A* 算法（修正版）
# ──────────────────────────────────────────────

def astar_search(costmap: SimpleCostmap2D, planner_cfg: PlannerConfig,
                 start_world: Tuple[float, float],
                 goal_world: Tuple[float, float]) -> SearchResult:
    """
    运行 A* 搜索。修正了 C++ 版本中的以下 bug：
    1. isOccupied 检查方向修正（跳过障碍物而非自由空间）
    2. brace 结构修正（cost computation 不被 if 包裹）
    3. closed_set 管理修正（pop 时标记，非 neighbor 遍历时）
    """
    result = SearchResult()
    res = planner_cfg.resolution

    # 转换为栅格坐标
    start_idx = costmap.world_to_grid(*start_world)
    goal_idx = costmap.world_to_grid(*goal_world)

    # 检查 start/goal
    sx, sy = start_world
    gx, gy = goal_world
    if not costmap.is_in_bounds(sx, sy):
        print('[A*] start 超出地图范围')
        return result
    if costmap.is_occupied(sx, sy):
        print('[A*] start 在障碍物中')
        return result
    if not costmap.is_in_bounds(gx, gy):
        print('[A*] goal 超出地图范围')
        return result
    if costmap.is_occupied(gx, gy):
        print('[A*] goal 在障碍物中')
        return result

    # 8 邻域方向
    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1)]

    def heuristic(a, b):
        return math.hypot(a.x - b.x, a.y - b.y) * res

    # A* 数据结构
    g_score = {start_idx: 0.0}
    came_from = {}
    closed_set = set()
    open_set = []  # (f_score, tie_breaker, GridIndex)
    counter = 0
    heapq.heappush(open_set, (planner_cfg.heuristic_weight * heuristic(start_idx, goal_idx), counter, start_idx))

    # 记录搜索过程
    open_set_snapshot = []
    closed_set_snapshot = []

    t0 = time.perf_counter()
    iterations = 0

    while open_set and iterations < planner_cfg.max_iterations:
        iterations += 1
        f, _, current = heapq.heappop(open_set)

        # 到达目标
        if current == goal_idx:
            result.elapsed_ms = (time.perf_counter() - t0) * 1000
            result.iterations = iterations
            result.success = True
            result.open_set = open_set_snapshot.copy()
            result.closed_set = closed_set_snapshot.copy()

            # 回溯路径
            path_indices = []
            node = goal_idx
            while node != start_idx:
                path_indices.append(node)
                node = came_from.get(node)
                if node is None:
                    break
            path_indices.append(start_idx)
            path_indices.reverse()

            result.path = [costmap.grid_to_world(idx) for idx in path_indices]
            result.path_length = sum(
                math.hypot(result.path[i + 1][0] - result.path[i][0],
                           result.path[i + 1][1] - result.path[i][1])
                for i in range(len(result.path) - 1)
            )
            result.path_cost = g_score.get(goal_idx, 0.0)
            return result

        if current in closed_set:
            continue
        closed_set.add(current)
        closed_set_snapshot.append(current)

        # 扩展邻居
        for dx, dy in dirs:
            neighbor = GridIndex(current.x + dx, current.y + dy)

            if neighbor in closed_set:
                continue

            # 获取邻居的世界坐标
            wx, wy = costmap.grid_to_world(neighbor)

            # 边界检查
            if not costmap.is_in_bounds(wx, wy):
                continue

            # 障碍物检查（修正：跳过被占据的格子，而非自由格子）
            if costmap.is_occupied(wx, wy):
                continue

            # 未知区域检查
            if not planner_cfg.allow_unknown and costmap.get_cost(wx, wy) == 255:
                continue

            # 计算移动代价
            adx = abs(neighbor.x - current.x)
            ady = abs(neighbor.y - current.y)
            move_cost = math.sqrt(adx * adx + ady * ady)

            # 代价因子
            cell_cost = costmap.get_cost(wx, wy)
            cost_factor = 1.0 + float(cell_cost) / 252.0

            tentative_g = g_score[current] + move_cost * cost_factor

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + planner_cfg.heuristic_weight * heuristic(neighbor, goal_idx)
                counter += 1
                heapq.heappush(open_set, (f_score, counter, neighbor))
                open_set_snapshot.append(neighbor)

    # 未找到路径
    result.elapsed_ms = (time.perf_counter() - t0) * 1000
    result.iterations = iterations
    result.open_set = open_set_snapshot.copy()
    result.closed_set = closed_set_snapshot.copy()
    return result


# ──────────────────────────────────────────────
# 配置文件读取
# ──────────────────────────────────────────────

def load_config(config_path: str) -> Tuple[CostmapConfig, PlannerConfig]:
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)

    params = data.get('nav_server', {}).get('ros__parameters', {})

    # 代价地图配置
    cm = params.get('costmap', {})
    raw_obs = cm.get('obstacles', [])
    obstacles = []
    for i in range(0, len(raw_obs) - 3, 4):
        obstacles.append((
            min(raw_obs[i], raw_obs[i + 2]),
            min(raw_obs[i + 1], raw_obs[i + 3]),
            max(raw_obs[i], raw_obs[i + 2]),
            max(raw_obs[i + 1], raw_obs[i + 3]),
        ))

    costmap_cfg = CostmapConfig(
        origin_x=cm.get('origin_x', -5.0),
        origin_y=cm.get('origin_y', -5.0),
        width=cm.get('width', 10.0),
        height=cm.get('height', 10.0),
        resolution=cm.get('resolution', 0.1),
        lethal_radius=cm.get('robot_radius', 0.25),
        inflation_radius=cm.get('inflation_radius', 0.5),
        obstacles=obstacles,
    )

    # 规划器配置
    gp = params.get('global_planner', {})
    planner_cfg = PlannerConfig(
        resolution=gp.get('resolution', costmap_cfg.resolution),
        heuristic_weight=gp.get('heuristic_weight', 1.2),
        max_iterations=gp.get('max_iterations', 100000),
        allow_unknown=gp.get('allow_unknown', False),
        diagonal_movement=gp.get('diagonal_movement', True),
    )

    return costmap_cfg, planner_cfg


# ──────────────────────────────────────────────
# 可视化主类
# ──────────────────────────────────────────────

class AStarVisualizer:
    def __init__(self, costmap_cfg: CostmapConfig, planner_cfg: PlannerConfig):
        self.costmap = SimpleCostmap2D(costmap_cfg)
        self.planner_cfg = planner_cfg
        self.costmap_cfg = costmap_cfg

        self.start_pos = None  # (x, y)
        self.goal_pos = None   # (x, y)
        self.search_result = None
        self.show_animation = False
        self.animation_idx = 0
        self.animation_timer = None

        self._setup_figure()
        self._redraw()

    def _setup_figure(self):
        self.fig, self.ax = plt.subplots(1, 1, figsize=(12, 10))
        self.fig.canvas.manager.set_window_title('A* 路径规划可视化')
        self.ax.set_title('左键=Start | 右键=Goal | a=动画 | c=清除 | q=退出')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_aspect('equal')

        cfg = self.costmap_cfg
        self.ax.set_xlim(cfg.origin_x, cfg.origin_x + cfg.width)
        self.ax.set_ylim(cfg.origin_y, cfg.origin_y + cfg.height)
        self.ax.grid(True, alpha=0.2)

        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

    def _redraw(self):
        self.ax.cla()

        cfg = self.costmap_cfg
        self.ax.set_xlim(cfg.origin_x, cfg.origin_x + cfg.width)
        self.ax.set_ylim(cfg.origin_y, cfg.origin_y + cfg.height)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.2)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')

        # 画代价热力图
        extent = [cfg.origin_x, cfg.origin_x + cfg.width,
                  cfg.origin_y, cfg.origin_y + cfg.height]
        self.ax.imshow(self.costmap.cost_grid.T, origin='lower', extent=extent,
                       cmap='RdYlGn_r', vmin=0, vmax=254, alpha=0.7,
                       interpolation='bilinear')

        # 画障碍物轮廓
        for box in cfg.obstacles:
            min_x, min_y, max_x, max_y = box
            rect = Rectangle((min_x, min_y), max_x - min_x, max_y - min_y,
                              linewidth=2, edgecolor='darkred', facecolor='none',
                              linestyle='-')
            self.ax.add_patch(rect)

        # 画搜索过程
        if self.search_result and self.show_animation:
            self._draw_search_process()

        # 画路径
        if self.search_result and self.search_result.success:
            path = self.search_result.path
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            self.ax.plot(px, py, 'w-', linewidth=2.5, zorder=5, label='路径')
            self.ax.plot(px, py, 'o', color='yellow', markersize=3, zorder=6)

        # 画 start / goal
        if self.start_pos:
            self.ax.plot(self.start_pos[0], self.start_pos[1], '^',
                         color='lime', markersize=15, markeredgecolor='black',
                         markeredgewidth=1.5, zorder=10, label='Start')
        if self.goal_pos:
            self.ax.plot(self.goal_pos[0], self.goal_pos[1], '*',
                         color='red', markersize=18, markeredgecolor='black',
                         markeredgewidth=1.5, zorder=10, label='Goal')

        # 标题信息
        title = '左键=Start | 右键=Goal | a=动画 | c=清除 | q=退出'
        if self.search_result:
            r = self.search_result
            if r.success:
                title += f'\n路径: {r.path_length:.2f}m | 代价: {r.path_cost:.2f} | 迭代: {r.iterations} | 耗时: {r.elapsed_ms:.1f}ms'
            else:
                title += f'\n未找到路径 | 迭代: {r.iterations} | 耗时: {r.elapsed_ms:.1f}ms'
        self.ax.set_title(title)
        self.ax.legend(loc='upper right', fontsize=9)
        self.fig.canvas.draw_idle()

    def _draw_search_process(self):
        """画 A* 搜索过程"""
        if not self.search_result:
            return
        res = self.search_result
        cfg = self.costmap_cfg

        # 画 closed_set（已探索）
        for idx in res.closed_set:
            wx = (idx.x + 0.5) * cfg.resolution
            wy = (idx.y + 0.5) * cfg.resolution
            self.ax.plot(wx, wy, 's', color='lightblue', markersize=2,
                         alpha=0.4, zorder=2)

        # 画 open_set（待探索）
        for idx in res.open_set:
            wx = (idx.x + 0.5) * cfg.resolution
            wy = (idx.y + 0.5) * cfg.resolution
            self.ax.plot(wx, wy, 's', color='orange', markersize=2,
                         alpha=0.4, zorder=3)

    def _on_click(self, event):
        if event.inaxes != self.ax:
            return

        if event.button == 1:  # 左键：设置 start
            self.start_pos = (event.xdata, event.ydata)
            self.search_result = None
            if self.start_pos and self.goal_pos:
                self._run_astar()
            else:
                self._redraw()

        elif event.button == 3:  # 右键：设置 goal
            self.goal_pos = (event.xdata, event.ydata)
            self.search_result = None
            if self.start_pos and self.goal_pos:
                self._run_astar()
            else:
                self._redraw()

    def _on_key(self, event):
        if event.key == 'q':
            plt.close(self.fig)
        elif event.key == 'c':
            self.start_pos = None
            self.goal_pos = None
            self.search_result = None
            self.show_animation = False
            self._redraw()
        elif event.key == 'a':
            self.show_animation = not self.show_animation
            self._redraw()

    def _run_astar(self):
        if not self.start_pos or not self.goal_pos:
            return

        # 使用规划器分辨率（可能与代价地图不同）
        planner_cfg = PlannerConfig(
            resolution=self.planner_cfg.resolution,
            heuristic_weight=self.planner_cfg.heuristic_weight,
            max_iterations=self.planner_cfg.max_iterations,
            allow_unknown=self.planner_cfg.allow_unknown,
            diagonal_movement=self.planner_cfg.diagonal_movement,
        )

        print(f'[A*] Start: ({self.start_pos[0]:.2f}, {self.start_pos[1]:.2f})')
        print(f'[A*] Goal:  ({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f})')

        self.search_result = astar_search(
            self.costmap, planner_cfg, self.start_pos, self.goal_pos)

        r = self.search_result
        if r.success:
            print(f'[A*] 找到路径! 长度={r.path_length:.2f}m, 代价={r.path_cost:.2f}, '
                  f'迭代={r.iterations}, 耗时={r.elapsed_ms:.1f}ms, 路径点={len(r.path)}')
        else:
            print(f'[A*] 未找到路径. 迭代={r.iterations}, 耗时={r.elapsed_ms:.1f}ms')

        self._redraw()


# ──────────────────────────────────────────────
# 入口
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='A* 路径规划可视化工具')
    parser.add_argument('--config', type=str,
                        default='src/pnc_nav_bringup/config/nav_params.yaml',
                        help='nav_params.yaml 配置文件路径')
    args = parser.parse_args()

    try:
        costmap_cfg, planner_cfg = load_config(args.config)
    except FileNotFoundError:
        print(f'错误: 找不到配置文件 {args.config}')
        print('请用 --config 指定 nav_params.yaml 的路径')
        sys.exit(1)
    except Exception as e:
        print(f'错误: 解析配置文件失败: {e}')
        sys.exit(1)

    print(f'加载配置: {args.config}')
    print(f'  地图: {costmap_cfg.width}m x {costmap_cfg.height}m, '
          f'分辨率={costmap_cfg.resolution}m')
    print(f'  障碍物: {len(costmap_cfg.obstacles)} 个矩形')
    print(f'  规划器: A*, 分辨率={planner_cfg.resolution}m, '
          f'启发式权重={planner_cfg.heuristic_weight}')
    print()

    viz = AStarVisualizer(costmap_cfg, planner_cfg)
    plt.show()


if __name__ == '__main__':
    main()
