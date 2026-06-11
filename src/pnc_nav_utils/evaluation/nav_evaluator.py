#!/usr/bin/env python3
"""
导航性能评估工具

评估指标：
- 路径长度 (m)
- 路径平滑度 (曲率积分)
- 实际行驶距离 (m)
- 总导航时间 (s)
- 高度变化 (m)

用法（在线模式，实时评估）:
  python3 src/pnc_nav_utils/evaluation/nav_evaluator.py
  # Ctrl+C 退出时保存结果到 nav_benchmark.csv
"""

import math
import time
from dataclasses import dataclass, field
from typing import List

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped


@dataclass
class NavMetrics:
    """单次导航的评估指标"""
    path_length: float = 0.0           # 规划路径长度 (m)
    actual_distance: float = 0.0       # 实际行驶距离 (m)
    planning_time_ms: float = 0.0      # 规划耗时 (ms)
    total_time_s: float = 0.0          # 总导航时间 (s)
    smoothness: float = 0.0            # 路径平滑度 (曲率积分)
    max_curvature: float = 0.0         # 最大曲率
    cross_track_error_avg: float = 0.0 # 平均横向误差 (m)
    cross_track_error_max: float = 0.0 # 最大横向误差 (m)
    success: bool = False              # 是否成功到达
    height_change: float = 0.0         # 总高度变化 (m)


@dataclass
class BenchmarkResult:
    """多次导航的汇总结果"""
    planner_name: str = ""
    num_trials: int = 0
    success_rate: float = 0.0
    avg_path_length: float = 0.0
    avg_planning_time_ms: float = 0.0
    avg_total_time_s: float = 0.0
    avg_smoothness: float = 0.0
    avg_cross_track_error: float = 0.0
    trials: List[NavMetrics] = field(default_factory=list)


def compute_path_length(path: Path) -> float:
    """计算路径长度"""
    length = 0.0
    poses = path.poses
    for i in range(1, len(poses)):
        dx = poses[i].pose.position.x - poses[i-1].pose.position.x
        dy = poses[i].pose.position.y - poses[i-1].pose.position.y
        dz = poses[i].pose.position.z - poses[i-1].pose.position.z
        length += math.sqrt(dx*dx + dy*dy + dz*dz)
    return length


def compute_smoothness(path: Path) -> float:
    """计算路径平滑度（曲率变化的积分）"""
    if len(path.poses) < 3:
        return 0.0

    total_curvature = 0.0
    poses = path.poses
    for i in range(1, len(poses) - 1):
        p0 = poses[i-1].pose.position
        p1 = poses[i].pose.position
        p2 = poses[i+1].pose.position

        v1 = (p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)
        v2 = (p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)

        dot = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
        mag1 = math.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2)
        mag2 = math.sqrt(v2[0]**2 + v2[1]**2 + v2[2]**2)

        if mag1 > 1e-6 and mag2 > 1e-6:
            cos_angle = max(-1.0, min(1.0, dot / (mag1 * mag2)))
            angle = math.acos(cos_angle)
            total_curvature += angle

    return total_curvature


def compute_height_change(path: Path) -> float:
    """计算路径总高度变化"""
    total = 0.0
    poses = path.poses
    for i in range(1, len(poses)):
        total += abs(poses[i].pose.position.z - poses[i-1].pose.position.z)
    return total


class NavEvaluator(Node):
    """导航评估节点"""

    def __init__(self):
        super().__init__('nav_evaluator')

        self.declare_parameter('live', True)
        self.declare_parameter('output_file', 'nav_benchmark.csv')

        self.current_metrics = NavMetrics()
        self.results: List[NavMetrics] = []
        self.nav_start_time = None
        self.last_odom_pose = None

        self.global_plan_sub = self.create_subscription(
            Path, 'global_plan', self.global_plan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.get_logger().info('NavEvaluator started')

    def global_plan_callback(self, msg: Path):
        """收到全局路径时记录"""
        # 保存前一次的结果
        if self.nav_start_time is not None:
            self.current_metrics.total_time_s = time.time() - self.nav_start_time
            self.results.append(self.current_metrics)

        # 开始新的评估
        self.current_metrics = NavMetrics()
        self.current_metrics.path_length = compute_path_length(msg)
        self.current_metrics.smoothness = compute_smoothness(msg)
        self.current_metrics.height_change = compute_height_change(msg)
        self.nav_start_time = time.time()
        self.last_odom_pose = None

        self.get_logger().info(
            f'New plan: length={self.current_metrics.path_length:.2f}m, '
            f'smoothness={self.current_metrics.smoothness:.3f}, '
            f'height_change={self.current_metrics.height_change:.2f}m')

    def odom_callback(self, msg: Odometry):
        """累计实际行驶距离"""
        pos = msg.pose.pose.position
        if self.last_odom_pose is not None:
            dx = pos.x - self.last_odom_pose[0]
            dy = pos.y - self.last_odom_pose[1]
            dz = pos.z - self.last_odom_pose[2]
            self.current_metrics.actual_distance += math.sqrt(dx*dx + dy*dy + dz*dz)
        self.last_odom_pose = (pos.x, pos.y, pos.z)

    def save_results(self):
        """保存评估结果到CSV"""
        # 保存当前正在进行的评估
        if self.nav_start_time is not None:
            self.current_metrics.total_time_s = time.time() - self.nav_start_time
            self.results.append(self.current_metrics)

        if not self.results:
            self.get_logger().warn('没有评估数据，跳过保存')
            return

        output_file = self.get_parameter('output_file').value
        with open(output_file, 'w') as f:
            f.write('trial,path_length,actual_distance,total_time_s,smoothness,height_change\n')
            for i, m in enumerate(self.results):
                f.write(f'{i},{m.path_length:.3f},{m.actual_distance:.3f},'
                        f'{m.total_time_s:.1f},{m.smoothness:.4f},{m.height_change:.3f}\n')
        self.get_logger().info(f'Results saved to {output_file} ({len(self.results)} trials)')


def main(args=None):
    rclpy.init(args=args)
    node = NavEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_results()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
