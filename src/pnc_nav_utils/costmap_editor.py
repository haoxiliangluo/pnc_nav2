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
from matplotlib.collections import PatchCollection


# ──────────────────────────────────────────────
# 代价计算（复现 SimpleCostmap2D 逻辑）
# ──────────────────────────────────────────────

def distance_to_box(x, y, box):
    """点到轴对齐矩形的距离（与 C++ distanceToBox 一致）"""
    dx = max(box[0] - x, 0.0, x - box[2])
    dy = max(box[1] - y, 0.0, y - box[3])
    return np.hypot(dx, dy)


def compute_cost_grid(obstacles, origin_x, origin_y, width, height,
                      resolution, lethal_radius, inflation_radius):
    """生成代价网格（与 SimpleCostmap2D::getCost 逻辑一致）"""
    nx = int(round(width / resolution))
    ny = int(round(height / resolution))
    grid = np.zeros((ny, nx), dtype=np.float64)

    for iy in range(ny):
        for ix in range(nx):
            x = origin_x + (ix + 0.5) * resolution
            y = origin_y + (iy + 0.5) * resolution

            if not obstacles:
                grid[iy, ix] = 0.0
                continue

            dist = min(distance_to_box(x, y, b) for b in obstacles)
            if dist <= lethal_radius:
                grid[iy, ix] = 254.0  # LETHAL
            elif dist <= inflation_radius:
                ratio = 1.0 - (dist - lethal_radius) / max(1e-6, inflation_radius - lethal_radius)
                grid[iy, ix] = np.clip(180.0 * ratio, 1.0, 180.0)
            else:
                grid[iy, ix] = 0.0  # FREE_SPACE

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

        self._setup_figure()
        self._redraw()

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

    def _redraw(self):
        self.ax.cla()
        self.ax.set_title(
            f'障碍物: {len(self.obstacles)} | 左键拖拽画 | 右键删 | s保存 | c清空 | q退出')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_aspect('equal')
        self.ax.set_xlim(self.origin_x, self.origin_x + self.width)
        self.ax.set_ylim(self.origin_y, self.origin_y + self.height)
        self.ax.grid(True, alpha=0.3)

        # 画代价热力图
        cost_grid = compute_cost_grid(
            self.obstacles, self.origin_x, self.origin_y,
            self.width, self.height, self.resolution,
            self.lethal_radius, self.inflation_radius)

        extent = [self.origin_x, self.origin_x + self.width,
                  self.origin_y, self.origin_y + self.height]
        self.ax.imshow(cost_grid.T, origin='lower', extent=extent,
                       cmap='RdYlGn_r', vmin=0, vmax=254, alpha=0.6,
                       interpolation='bilinear')

        # 画障碍物矩形
        for box in self.obstacles:
            min_x, min_y, max_x, max_y = box
            rect = Rectangle((min_x, min_y), max_x - min_x, max_y - min_y,
                              linewidth=2, edgecolor='red', facecolor='darkred',
                              alpha=0.8)
            self.ax.add_patch(rect)

        self.fig.canvas.draw_idle()

    def _on_press(self, event):
        if event.inaxes != self.ax:
            return
        if event.button == 1:  # 左键：开始画矩形
            self.drag_start = (event.xdata, event.ydata)
        elif event.button == 3:  # 右键：删除最近障碍物
            self._delete_nearest(event.xdata, event.ydata)

    def _on_motion(self, event):
        if self.drag_start is None or event.inaxes != self.ax:
            return
        # 画临时矩形
        if self.drag_rect is not None:
            self.drag_rect.remove()
        x0, y0 = self.drag_start
        x1, y1 = event.xdata, event.ydata
        min_x, max_x = min(x0, x1), max(x0, x1)
        min_y, max_y = min(y0, y1), max(y0, y1)
        self.drag_rect = Rectangle((min_x, min_y), max_x - min_x, max_y - min_y,
                                    linewidth=2, edgecolor='blue',
                                    facecolor='blue', alpha=0.3,
                                    linestyle='--')
        self.ax.add_patch(self.drag_rect)
        self.fig.canvas.draw_idle()

    def _on_release(self, event):
        if self.drag_start is None or event.inaxes != self.ax:
            self.drag_start = None
            return
        x0, y0 = self.drag_start
        x1, y1 = event.xdata, event.ydata
        self.drag_start = None
        if self.drag_rect is not None:
            self.drag_rect.remove()
            self.drag_rect = None

        min_x, max_x = min(x0, x1), max(x0, x1)
        min_y, max_y = min(y0, y1), max(y0, y1)

        # 忽略太小的矩形（可能是误触）
        if max_x - min_x < self.resolution * 2 or max_y - min_y < self.resolution * 2:
            return

        self.obstacles.append((min_x, min_y, max_x, max_y))
        self._redraw()

    def _delete_nearest(self, x, y):
        if not self.obstacles:
            return
        # 找中心距离最近的障碍物
        best_idx = min(range(len(self.obstacles)),
                       key=lambda i: ((self.obstacles[i][0] + self.obstacles[i][2]) / 2 - x) ** 2 +
                                     ((self.obstacles[i][1] + self.obstacles[i][3]) / 2 - y) ** 2)
        self.obstacles.pop(best_idx)
        self._redraw()

    def _on_key(self, event):
        if event.key == 'q':
            plt.close(self.fig)
        elif event.key == 'c':
            self.obstacles.clear()
            self._redraw()
        elif event.key == 'r':
            self._redraw()
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

        # 同时保存到文件
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
