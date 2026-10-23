#!/usr/bin/env python3
"""订阅一次 /costmap 与 /obstacle_layer，打印尺寸/原点/占用格数，并对比两次采样验证滚动。"""
import sys
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy


class Checker(Node):
    def __init__(self):
        super().__init__('costmap_checker')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.costmap = None
        self.obstacle = None
        self.create_subscription(OccupancyGrid, '/costmap', self.cb_cost, qos)
        self.create_subscription(OccupancyGrid, '/obstacle_layer', self.cb_obs, qos)

    def cb_cost(self, msg):
        self.costmap = msg

    def cb_obs(self, msg):
        self.obstacle = msg


def summarize(tag, msg):
    if msg is None:
        print(f'{tag}: no data')
        return
    occupied = sum(1 for v in msg.data if v >= 99)
    unknown = sum(1 for v in msg.data if v < 0)
    print(
        f'{tag}: {msg.info.width}x{msg.info.height} res={msg.info.resolution:.2f} '
        f'origin=({msg.info.origin.position.x:.2f},{msg.info.origin.position.y:.2f}) '
        f'cells={len(msg.data)} occupied={occupied} unknown={unknown}')


def main():
    rclpy.init()
    node = Checker()
    deadline = time.time() + 8
    while time.time() < deadline and (node.costmap is None or node.obstacle is None):
        rclpy.spin_once(node, timeout_sec=0.2)
    summarize('/costmap', node.costmap)
    summarize('/obstacle_layer', node.obstacle)
    # 记录 origin，便于外部对比
    if node.costmap:
        print(f'ORIGIN {node.costmap.info.origin.position.x:.3f} {node.costmap.info.origin.position.y:.3f}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
