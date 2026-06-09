#!/usr/bin/env python3
"""
C++ 全局规划器可视化测试工具

这个脚本通过 ROS2 话题与 NavServer 交互，真正调用你写的 C++ AStar2D 规划器。
不是用 Python 重写 A*，而是测试你 C++ 实现的规划器。

工作原理：
  1. 发布假的 TF (map -> base_link) 来设定机器人起点位置
  2. 发布 goal_pose 话题触发 NavServer 调用 C++ 规划器
  3. 订阅 global_plan 话题获取 C++ 规划器返回的路径
  4. 用 matplotlib 可视化代价地图 + 路径

用法：
  终端1: ros2 launch pnc_nav_bringup nav_bringup.launch.py
  终端2: python3 src/pnc_nav_utils/planner_tester.py

交互：
  左键点击  : 设置 start 点（绿色三角）→ 发布 TF
  右键点击  : 设置 goal 点（红色星）→ 触发 C++ 规划器
  c 键      : 清除路径和 start/goal
  q 键      : 退出
"""

import argparse
import math
import sys
import time
from typing import List, Optional, Tuple

import numpy as np
import yaml
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster


# ──────────────────────────────────────────────
# 代价地图（从 nav_params.yaml 构建，纯可视化用）
# ──────────────────────────────────────────────

class CostmapVisualizer:
    """从配置文件构建代价地图网格，仅用于可视化显示"""

    def __init__(self, origin_x, origin_y, width, height, resolution,
                 lethal_radius, inflation_radius, obstacles):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = width
        self.height = height
        self.resolution = resolution
        self.lethal_radius = lethal_radius
        self.inflation_radius = inflation_radius
        self.obstacles = obstacles

        self.nx = int(round(width / resolution))
        self.ny = int(round(height / resolution))
        self.cost_grid = self._build_cost_grid()

    def _distance_to_box(self, x, y, box):
        dx = max(box[0] - x, 0.0, x - box[2])
        dy = max(box[1] - y, 0.0, y - box[3])
        return math.hypot(dx, dy)

    def _build_cost_grid(self):
        grid = np.zeros((self.ny, self.nx), dtype=np.float64)
        for iy in range(self.ny):
            for ix in range(self.nx):
                x = self.origin_x + (ix + 0.5) * self.resolution
                y = self.origin_y + (iy + 0.5) * self.resolution
                if not self.obstacles:
                    grid[iy, ix] = 0.0
                    continue
                dist = min(self._distance_to_box(x, y, b) for b in self.obstacles)
                if dist <= self.lethal_radius:
                    grid[iy, ix] = 254.0
                elif dist <= self.inflation_radius:
                    ratio = 1.0 - (dist - self.lethal_radius) / max(1e-6, self.inflation_radius - self.lethal_radius)
                    grid[iy, ix] = np.clip(180.0 * ratio, 1.0, 180.0)
                else:
                    grid[iy, ix] = 0.0
        return grid


# ──────────────────────────────────────────────
# ROS2 节点：与 NavServer 交互
# ──────────────────────────────────────────────

class PlannerTestNode(Node):
    """ROS2 节点，通过话题调用 NavServer 的 C++ 规划器"""

    def __init__(self):
        super().__init__('planner_tester')

        # 发布 goal_pose 触发规划
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # 发布 TF (map -> base_link) 来设定起点
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅 global_plan 获取 C++ 规划器的结果
        self.plan_sub = self.create_subscription(
            Path, 'global_plan', self._plan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))

        self.latest_path = None
        self.path_timestamp = None

        self.get_logger().info('PlannerTester 节点已启动，等待交互...')

    def publish_start_tf(self, x: float, y: float):
        """发布 TF 变换，将机器人放置在指定位置作为起点"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'发布 TF: robot at ({x:.2f}, {y:.2f})')

    def send_goal(self, x: float, y: float):
        """发布目标点，触发 NavServer 调用 C++ 规划器"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.latest_path = None  # 清除旧路径
        self.get_logger().info(f'发送目标: ({x:.2f}, {y:.2f})，等待 C++ 规划器响应...')

    def _plan_callback(self, msg: Path):
        """收到 C++ 规划器发布的路径"""
        self.latest_path = msg
        self.path_timestamp = time.time()
        n = len(msg.poses)
        if n > 0:
            # 计算路径长度
            length = 0.0
            for i in range(n - 1):
                dx = msg.poses[i+1].pose.position.x - msg.poses[i].pose.position.x
                dy = msg.poses[i+1].pose.position.y - msg.poses[i].pose.position.y
                length += math.hypot(dx, dy)
            self.get_logger().info(
                f'收到 C++ 规划器路径: {n} 个路径点, 总长 {length:.2f}m')
        else:
            self.get_logger().warn('C++ 规划器返回空路径（规划失败）')


# ──────────────────────────────────────────────
# 可视化主类
# ──────────────────────────────────────────────

class PlannerTesterVisualizer:
    """matplotlib 可视化 + ROS2 交互"""

    def __init__(self, node: PlannerTestNode, costmap_cfg: dict):
        self.node = node
        self.costmap = CostmapVisualizer(**costmap_cfg)

        self.start_pos = None   # (x, y)
        self.goal_pos = None    # (x, y)
        self.cpp_path = None    # [(x,y), ...] from C++ planner

        self._setup_figure()
        self._redraw()

    def _setup_figure(self):
        self.fig, self.ax = plt.subplots(1, 1, figsize=(12, 10))
        self.fig.canvas.manager.set_window_title('C++ AStar2D 规划器测试')
        self.ax.set_title('左键=Start | 右键=Goal(触发C++规划) | c=清除 | q=退出')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_aspect('equal')

        cm = self.costmap
        self.ax.set_xlim(cm.origin_x, cm.origin_x + cm.width)
        self.ax.set_ylim(cm.origin_y, cm.origin_y + cm.height)
        self.ax.grid(True, alpha=0.2)

        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

        # 定时器：轮询 ROS2 回调 + 检查新路径
        self.timer = self.fig.canvas.new_timer(interval=50)  # 50ms = 20Hz
        self.timer.add_callback(self._ros_spin)
        self.timer.start()

    def _ros_spin(self):
        """定时处理 ROS2 回调并检查新路径"""
        rclpy.spin_once(self.node, timeout_sec=0)

        # 检查是否有新路径
        if self.node.latest_path is not None:
            path_msg = self.node.latest_path
            self.node.latest_path = None  # 消费掉

            if len(path_msg.poses) > 0:
                self.cpp_path = [
                    (p.pose.position.x, p.pose.position.y)
                    for p in path_msg.poses
                ]
                print(f'[C++ AStar2D] 收到路径: {len(self.cpp_path)} 个点')
            else:
                self.cpp_path = []
                print('[C++ AStar2D] 规划失败: 返回空路径')

            self._redraw()

    def _redraw(self):
        self.ax.cla()

        cm = self.costmap
        self.ax.set_xlim(cm.origin_x, cm.origin_x + cm.width)
        self.ax.set_ylim(cm.origin_y, cm.origin_y + cm.height)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.2)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')

        # 代价热力图
        extent = [cm.origin_x, cm.origin_x + cm.width,
                  cm.origin_y, cm.origin_y + cm.height]
        self.ax.imshow(cm.cost_grid.T, origin='lower', extent=extent,
                       cmap='RdYlGn_r', vmin=0, vmax=254, alpha=0.7,
                       interpolation='bilinear')

        # 障碍物轮廓
        for box in cm.obstacles:
            min_x, min_y, max_x, max_y = box
            rect = Rectangle((min_x, min_y), max_x - min_x, max_y - min_y,
                              linewidth=2, edgecolor='darkred', facecolor='none',
                              linestyle='-')
            self.ax.add_patch(rect)

        # C++ 规划器返回的路径
        if self.cpp_path and len(self.cpp_path) > 1:
            px = [p[0] for p in self.cpp_path]
            py = [p[1] for p in self.cpp_path]
            self.ax.plot(px, py, 'w-', linewidth=2.5, zorder=5,
                         label=f'C++ A* 路径 ({len(self.cpp_path)}点)')
            self.ax.plot(px, py, 'o', color='yellow', markersize=3, zorder=6)

        # start / goal
        if self.start_pos:
            self.ax.plot(self.start_pos[0], self.start_pos[1], '^',
                         color='lime', markersize=15, markeredgecolor='black',
                         markeredgewidth=1.5, zorder=10, label='Start')
        if self.goal_pos:
            self.ax.plot(self.goal_pos[0], self.goal_pos[1], '*',
                         color='red', markersize=18, markeredgecolor='black',
                         markeredgewidth=1.5, zorder=10, label='Goal')

        # 标题
        title = '左键=Start | 右键=Goal(触发C++规划) | c=清除 | q=退出'
        if self.cpp_path is not None:
            if len(self.cpp_path) > 1:
                length = sum(
                    math.hypot(self.cpp_path[i+1][0] - self.cpp_path[i][0],
                               self.cpp_path[i+1][1] - self.cpp_path[i][1])
                    for i in range(len(self.cpp_path) - 1)
                )
                title += f'\nC++ A*2D: {len(self.cpp_path)} 点, 路径长 {length:.2f}m'
            else:
                title += '\nC++ A*2D: 规划失败'
        self.ax.set_title(title)
        self.ax.legend(loc='upper right', fontsize=9)
        self.fig.canvas.draw_idle()

    def _on_click(self, event):
        if event.inaxes != self.ax:
            return

        if event.button == 1:  # 左键：设置 start
            self.start_pos = (event.xdata, event.ydata)
            self.cpp_path = None
            self.node.publish_start_tf(self.start_pos[0], self.start_pos[1])
            print(f'[Start] ({self.start_pos[0]:.2f}, {self.start_pos[1]:.2f})')
            self._redraw()

        elif event.button == 3:  # 右键：设置 goal，触发 C++ 规划器
            self.goal_pos = (event.xdata, event.ydata)
            self.cpp_path = None

            if not self.start_pos:
                print('[提示] 请先左键设置 Start 点')
                self._redraw()
                return

            # 持续发布 TF（确保 NavServer 能拿到起点）
            self.node.publish_start_tf(self.start_pos[0], self.start_pos[1])
            # 发送目标，触发 C++ AStar2D 规划器
            self.node.send_goal(self.goal_pos[0], self.goal_pos[1])
            print(f'[Goal]  ({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f}) → 等待 C++ 规划器...')
            self._redraw()

    def _on_key(self, event):
        if event.key == 'q':
            plt.close(self.fig)
        elif event.key == 'c':
            self.start_pos = None
            self.goal_pos = None
            self.cpp_path = None
            print('[清除] 已清除所有标记和路径')
            self._redraw()


# ──────────────────────────────────────────────
# 配置读取
# ──────────────────────────────────────────────

def load_costmap_config(config_path: str) -> dict:
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)

    params = data.get('nav_server', {}).get('ros__parameters', {})
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

    return dict(
        origin_x=cm.get('origin_x', -5.0),
        origin_y=cm.get('origin_y', -5.0),
        width=cm.get('width', 10.0),
        height=cm.get('height', 10.0),
        resolution=cm.get('resolution', 0.1),
        lethal_radius=cm.get('robot_radius', 0.25),
        inflation_radius=cm.get('inflation_radius', 0.5),
        obstacles=obstacles,
    )


# ──────────────────────────────────────────────
# 入口
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='C++ AStar2D 规划器可视化测试工具')
    parser.add_argument('--config', type=str,
                        default='src/pnc_nav_bringup/config/nav_params.yaml',
                        help='nav_params.yaml 配置文件路径')
    args = parser.parse_args()

    # 加载配置
    try:
        costmap_cfg = load_costmap_config(args.config)
    except FileNotFoundError:
        print(f'错误: 找不到配置文件 {args.config}')
        sys.exit(1)
    except Exception as e:
        print(f'错误: 解析配置文件失败: {e}')
        sys.exit(1)

    print(f'代价地图: {costmap_cfg["width"]}m x {costmap_cfg["height"]}m, '
          f'分辨率={costmap_cfg["resolution"]}m, '
          f'障碍物={len(costmap_cfg["obstacles"])}个')
    print()

    # 初始化 ROS2
    rclpy.init()
    node = PlannerTestNode()

    print('='*50)
    print('C++ AStar2D 规划器测试工具')
    print('='*50)
    print('请确保 NavServer 已启动:')
    print('  ros2 launch pnc_nav_bringup nav_bringup.launch.py')
    print()
    print('交互方式:')
    print('  左键点击 → 设置 start 点 (发布 TF)')
    print('  右键点击 → 设置 goal 点 (触发 C++ 规划器)')
    print('  c 键 → 清除')
    print('  q 键 → 退出')
    print('='*50)
    print()

    # 启动可视化
    viz = PlannerTesterVisualizer(node, costmap_cfg)
    plt.show()

    # 清理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
