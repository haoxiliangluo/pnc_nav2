#!/usr/bin/env python3
"""Monotonize laser scan timestamps.

gz gpu_lidar can publish multiple LaserScan messages with the SAME header
stamp (render-loop quirk). Cartographer requires strictly increasing
timestamps and aborts otherwise. This node re-publishes /nav_scan as
/nav_scan_mono with stamps forced to be strictly increasing (+1ns).

Usage (inside container):
  export ROS_DOMAIN_ID=43
  python3 <pkg>/scripts/mono_scan_node.py
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class MonoScanNode(Node):
    def __init__(self):
        super().__init__('mono_scan_node')
        self.last_stamp_ns = None
        # Drop messages during warmup: gz gpu_lidar emits duplicate-stamp
        # scans right at sim start; cartographer aborts on equal stamps.
        self.warmup_until = time.monotonic() + 5.0
        self.sub = self.create_subscription(
            LaserScan, 'nav_scan', self.on_scan, 10)
        self.pub = self.create_publisher(LaserScan, 'nav_scan_mono', 10)

    def on_scan(self, msg):
        if time.monotonic() < self.warmup_until:
            return
        stamp = msg.header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.last_stamp_ns is not None and ns <= self.last_stamp_ns:
            ns = self.last_stamp_ns + 1
        self.last_stamp_ns = ns
        msg.header.stamp.sec = ns // 1_000_000_000
        msg.header.stamp.nanosec = ns % 1_000_000_000
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MonoScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
