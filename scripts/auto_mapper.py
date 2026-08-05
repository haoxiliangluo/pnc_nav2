#!/usr/bin/env python3
"""Auto mapper: square spiral coverage for Cartographer SLAM.

Publishes /cmd_vel to drive pnc_diff_drive in an expanding square spiral
(2m, 4m, 6m ... sides), which gives loop closures for pose-graph optimization.

Usage (inside container, after cartographer_slam launch is up):
  python3 /home/hao/pnc_nav2/scripts/auto_mapper.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def move(self, vx, wz, dur):
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        self.pub.publish(t)
        time.sleep(dur)

    def stop(self):
        self.move(0.0, 0.0, 0.5)

    def run(self):
        v = 0.2          # m/s
        w_turn = 0.4     # rad/s
        # 1) spin two full turns in place (yaw calibration)
        self.get_logger().info('Phase 1: spinning in place')
        self.move(0.0, 0.3, 21.0)   # ~2 turns
        self.stop()

        # 2) square spiral: sides 2,4,6,... meters, left turns
        self.get_logger().info('Phase 2: square spiral')
        side = 2.0
        for ring in range(5):
            for _ in range(4):
                self.move(v, 0.0, side / v)
                self.stop()
                self.move(0.0, w_turn, math.pi / 2.0 / w_turn)
                self.stop()
            side += 2.0
            self.get_logger().info(f'ring {ring + 1} done, side={side}')

        # 3) return to origin-ish: reverse the spiral is complex, just stop
        self.stop()
        self.get_logger().info('Mapping run finished')


def main():
    rclpy.init()
    node = AutoMapper()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
