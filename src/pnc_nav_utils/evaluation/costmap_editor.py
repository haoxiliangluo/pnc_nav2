#!/usr/bin/env python3
"""
交互式代价地图编辑器

功能：
  - 左键拖拽：画矩形障碍物
  - 右键点击：删除最近的障碍物
  - s 键：导出 obstacles 到 YAML 片段（标准输出 + 文件）
  - c 键：清空所有障碍物
  - r 键：重绘
  - q 键：退出

用法：
  python3 costmap_editor.py
  python3 costmap_editor.py --origin -5 -5 --size 10 10 --resolution 0.1
"""

import argparse
import sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


# ──────────────────────────────────────────────
# 代价计算（向量化版本，与 SimpleCostmap2D 逻辑一致）
# ──────────────────────────────────────────────

def compute_cost_grid(obstacles, origin_x, origin_y, width, height,
                      resolution, lethal_radius, inflation_radius):
    """生成代价网格 — numpy 向量化，比纯 Python 循环快 100+ 倍"""
    nx = int(round(width / resolution))
    ny = int(round(height / resolution))

    # 构建网格中心坐标 (ny, nx)
    ix = np.arange(nx)
    iy = np.arange(ny)
    cx = origin_x + (ix + 0.5) * resolution   # (nx,)
    cy = origin_y + (iy + 0.5) * resolution   # (ny,)
    grid_x, grid_y = np.meshgrid(cx, cy)       # (ny, nx)

    if not obstacles:
        return np.zeros((ny, nx), dtype=np.float64)

    # 计算每个格子到所有障碍物的距离，取最小值
    min_dist = np.full((ny, nx), np.inf)
    for (min_bx, min_by, max_bx, max_by) in obstacles:
        # 点到轴对齐矩形的距离（向量化）
        dx = np.maximum(np.maximum(min_bx - grid_x, 0.0), grid_x - max_bx)
        dy = np.maximum(np.maximum(min_by - grid_y, 0.0), grid_y - max_by)
        dist = np.hypot(dx, dy)
        min_dist = np.minimum(min_dist, dist)

    # 代价赋值
    grid = np.zeros((ny, nx), dtype=np.float64)
    lethal_mask = min_dist <= lethal_radius
    inflation_mask = (~lethal_mask) & (min_dist <= inflation_radius)

    grid[lethal_mask] = 254.0
    ratio = 1.0 - (min_dist[inflation_mask] - lethal_radius) / max(1e-6, inflation_radius - lethal_radius)
    grid[inflation_mask] = np.clip(180.0 * ratio, 1.0, 180.0)

    return grid


# ──────────────────────────────────────────────
# 编辑器主类
# ──────────────────────────────────────────────

class CostmapEditor:
    def __init__(self, origin_x, origin_y, width, height, resolution,
                 lethal_radius, inflation_radius):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = width
        self.height = height
        self.resolution = resolution
        self.lethal_radius = lethal_radius
        self.inflation_radius = inflation_radius

        self.obstacles = []  # list of (min_x, min_y, max_x, max_y)
        self.drag_start = None
        self.drag_rect = None

        # 预计算坐标网格（只算一次，用于 imshow extent）
        self.extent = [origin_x, origin_x + width,
                       origin_y, origin_y + height]

        self._setup_figure()
        self._draw_heatmap()   # 初始画一次热力图
        self._draw_overlays()  # 画障碍物矩形

    def _setup_figure(self):
        self.fig, self.ax = plt.subplots(1, 1, figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('代价地图编辑器')
        self.ax.set_title('左键拖拽画障碍 | 右键删除 | s保存 | c清空 | q退出')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_aspect('equal')
        self.ax.set_xlim(self.origin_x, self.origin_x + self.width)
        self.ax.set_ylim(self.origin_y, self.origin_y + self.height)
        self.ax.grid(True, alpha=0.3)

        # 连接事件
        self.fig.canvas.mpl_connect('button_press_event', self._on_press)
        self.fig.canvas.mpl_connect('button_release_event', self._on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

    # ── 分层绘制：热力图只在障碍物变化时重算，overlay 每次轻量刷新 ──

    def _draw_heatmap(self):
        """重算并绘制代价热力图（只在障碍物增删时调用）"""
        # 移除旧的热力图
        if hasattr(self, '_heatmap') and self._heatmap is not None:
            self._heatmap.remove()

        cost_grid = compute_cost_grid(
            self.obstacles, self.origin_x, self.origin_y,
            self.width, self.height, self.resolution,
            self.lethal_radius, self.inflation_radius)

        self._heatmap = self.ax.imshow(
            cost_grid, origin='lower', extent=self.extent,
            cmap='RdYlGn_r', vmin=0, vmax=254, alpha=0.6,
            interpolation='bilinear', zorder=0)

    def _draw_overlays(self):
        """重绘障碍物矩形和标题（轻量操作）"""
        # 移除旧的矩形
        if hasattr(self, '_obstacle_patches'):
            for p in self._obstacle_patches:
                p.remove()
        self._obstacle_patches = []

        for box in self.obstacles:
            min_x, min_y, max_x, max_y = box
            rect = Rectangle((min_x, min_y), max_x - min_x, max_y - min_y,
                              linewidth=2, edgecolor='red', facecolor='darkred',
                              alpha=0.8, zorder=2)
            self.ax.add_patch(rect)
            self._obstacle_patches.append(rect)

        self.ax.set_title(
            f'障碍物: {len(self.obstacles)} | 左键拖拽画 | 右键删 | s保存 | c清空 | q退出')
        self.fig.canvas.draw_idle()

    def _full_redraw(self):
        """障碍物变化后：重算热力图 + 重绘 overlay"""
        self._draw_heatmap()
        self._draw_overlays()

    # ── 事件处理 ──

    def _on_press(self, event):
        if event.inaxes != self.ax:
            return
        if event.button == 1:  # 左键：开始画矩形
            self.drag_start = (event.xdata, event.ydata)
        elif event.button == 3:  # 右键：删除最近障碍物
            self._delete_nearest(event.xdata, event.ydata)

    def _on_motion(self, event):
        # 拖拽时只更新临时矩形，不重算热力图
        if self.drag_start is None or event.inaxes != self.ax:
            return
        if self.drag_rect is not None:
            self.drag_rect.remove()
        x0, y0 = self.drag_start
        x1, y1 = event.xdata, event.ydata
        min_x, max_x = min(x0, x1), max(x0, x1)
        min_y, max_y = min(y0, y1), max(y0, y1)
        self.drag_rect = Rectangle((min_x, min_y), max_x - min_x, max_y - min_y,
                                    linewidth=2, edgecolor='blue',
                                    facecolor='blue', alpha=0.3,
                                    linestyle='--', zorder=3)
        self.ax.add_patch(self.drag_rect)
        self.fig.canvas.draw_idle()

    def _on_release(self, event):
        if self.drag_start is None:
            return
        x0, y0 = self.drag_start
        self.drag_start = None

        # 移除临时矩形
        if self.drag_rect is not None:
            self.drag_rect.remove()
            self.drag_rect = None

        # 如果鼠标释放不在 axes 内，用事件坐标可能为 None
        if event.inaxes != self.ax or event.xdata is None:
            self.fig.canvas.draw_idle()
            return

        x1, y1 = event.xdata, event.ydata
        min_x, max_x = min(x0, x1), max(x0, x1)
        min_y, max_y = min(y0, y1), max(y0, y1)

        # 忽略太小的矩形（可能是误触）
        if max_x - min_x < self.resolution * 2 or max_y - min_y < self.resolution * 2:
            self.fig.canvas.draw_idle()
            return

        self.obstacles.append((min_x, min_y, max_x, max_y))
        self._full_redraw()

    def _delete_nearest(self, x, y):
        if not self.obstacles:
            return
        best_idx = min(range(len(self.obstacles)),
                       key=lambda i: ((self.obstacles[i][0] + self.obstacles[i][2]) / 2 - x) ** 2 +
                                     ((self.obstacles[i][1] + self.obstacles[i][3]) / 2 - y) ** 2)
        self.obstacles.pop(best_idx)
        self._full_redraw()

    def _on_key(self, event):
        if event.key == 'q':
            plt.close(self.fig)
        elif event.key == 'c':
            self.obstacles.clear()
            self._full_redraw()
        elif event.key == 'r':
            self._full_redraw()
        elif event.key == 's':
            self._export_yaml()

    def _export_yaml(self):
        if not self.obstacles:
            print('# 没有障碍物')
            return

        lines = []
        lines.append('      # 矩形障碍物: [min_x, min_y, max_x, max_y, ...]')
        lines.append('      obstacles: [')
        for i, (min_x, min_y, max_x, max_y) in enumerate(self.obstacles):
            comma = ',' if i < len(self.obstacles) - 1 else ''
            lines.append(f'        {min_x:.2f}, {min_y:.2f}, {max_x:.2f}, {max_y:.2f}{comma}')
        lines.append('      ]')

        yaml_text = '\n'.join(lines)
        print('\n' + '=' * 50)
        print('导出的 YAML 片段（可粘贴到 nav_params.yaml）:')
        print('=' * 50)
        print(yaml_text)
        print('=' * 50)

        output_file = 'costmap_obstacles.yaml'
        with open(output_file, 'w') as f:
            f.write(yaml_text + '\n')
        print(f'已保存到: {output_file}')


# ──────────────────────────────────────────────
# 入口
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='交互式代价地图编辑器')
    parser.add_argument('--origin', nargs=2, type=float, default=[-5.0, -5.0],
                        help='地图原点 (x y)，默认: -5 -5')
    parser.add_argument('--size', nargs=2, type=float, default=[10.0, 10.0],
                        help='地图尺寸 (width height)，默认: 10 10')
    parser.add_argument('--resolution', type=float, default=0.1,
                        help='栅格分辨率 (m)，默认: 0.1')
    parser.add_argument('--robot-radius', type=float, default=0.25,
                        help='机器人半径/致命半径 (m)，默认: 0.25')
    parser.add_argument('--inflation-radius', type=float, default=0.5,
                        help='膨胀半径 (m)，默认: 0.5')
    args = parser.parse_args()

    editor = CostmapEditor(
        origin_x=args.origin[0],
        origin_y=args.origin[1],
        width=args.size[0],
        height=args.size[1],
        resolution=args.resolution,
        lethal_radius=args.robot_radius,
        inflation_radius=args.inflation_radius,
    )
    plt.show()


if __name__ == '__main__':
    main()
