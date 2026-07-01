#!/usr/bin/env python3
"""
Record RViz 2D Nav Goal poses for repeatable navigation tests.

Typical use:
  python3 src/pnc_nav_utils/evaluation/goal_recorder.py

The script listens to /goal_pose and appends each goal to CSV and JSONL files.
If /odom is available, it also records the latest odom pose as the start pose at
the time the goal was received.
"""

import argparse
import csv
import json
import math
import os
import time
from dataclasses import asdict, dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class RecordedGoal:
    goal_id: int
    receive_time_unix: float
    msg_stamp_sec: int
    msg_stamp_nanosec: int
    goal_frame_id: str
    goal_x: float
    goal_y: float
    goal_z: float
    goal_yaw_rad: float
    goal_qx: float
    goal_qy: float
    goal_qz: float
    goal_qw: float
    start_available: bool
    start_frame_id: str = ""
    start_x: float = 0.0
    start_y: float = 0.0
    start_z: float = 0.0
    start_yaw_rad: float = 0.0


class GoalRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("goal_recorder")
        self.args = args
        self.goal_count = 0
        self.latest_odom: Optional[Odometry] = None

        self.create_subscription(PoseStamped, args.goal_topic, self.goal_callback, 10)
        if args.odom_topic:
            self.create_subscription(Odometry, args.odom_topic, self.odom_callback, 20)

        self.get_logger().info(
            f"Goal recorder started: goal_topic={args.goal_topic}, "
            f"odom_topic={args.odom_topic or 'disabled'}, csv={args.csv_output}, "
            f"jsonl={args.jsonl_output}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def goal_callback(self, msg: PoseStamped) -> None:
        self.goal_count += 1
        goal = msg.pose.position
        goal_q = msg.pose.orientation
        record = RecordedGoal(
            goal_id=self.goal_count,
            receive_time_unix=time.time(),
            msg_stamp_sec=msg.header.stamp.sec,
            msg_stamp_nanosec=msg.header.stamp.nanosec,
            goal_frame_id=msg.header.frame_id,
            goal_x=goal.x,
            goal_y=goal.y,
            goal_z=goal.z,
            goal_yaw_rad=yaw_from_quaternion(goal_q.x, goal_q.y, goal_q.z, goal_q.w),
            goal_qx=goal_q.x,
            goal_qy=goal_q.y,
            goal_qz=goal_q.z,
            goal_qw=goal_q.w,
            start_available=self.latest_odom is not None,
        )

        if self.latest_odom is not None:
            odom = self.latest_odom
            start = odom.pose.pose.position
            start_q = odom.pose.pose.orientation
            record.start_frame_id = odom.header.frame_id
            record.start_x = start.x
            record.start_y = start.y
            record.start_z = start.z
            record.start_yaw_rad = yaw_from_quaternion(
                start_q.x, start_q.y, start_q.z, start_q.w
            )

        self.write_record(record)
        self.get_logger().info(
            f"Recorded goal #{record.goal_id}: "
            f"goal=({record.goal_x:.3f}, {record.goal_y:.3f}, "
            f"yaw={record.goal_yaw_rad:.3f})"
        )

    def write_record(self, record: RecordedGoal) -> None:
        row = asdict(record)

        if self.args.csv_output:
            os.makedirs(os.path.dirname(self.args.csv_output) or ".", exist_ok=True)
            write_header = not os.path.exists(self.args.csv_output)
            with open(self.args.csv_output, "a", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=list(row.keys()))
                if write_header:
                    writer.writeheader()
                writer.writerow(row)

        if self.args.jsonl_output:
            os.makedirs(os.path.dirname(self.args.jsonl_output) or ".", exist_ok=True)
            with open(self.args.jsonl_output, "a", encoding="utf-8") as f:
                f.write(json.dumps(row, ensure_ascii=False) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record RViz /goal_pose messages for repeatable nav tests."
    )
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument(
        "--odom-topic",
        default="/odom",
        help="Latest odom pose is recorded as start pose. Use '' to disable.",
    )
    parser.add_argument("--csv-output", default="test/recorded_goals.csv")
    parser.add_argument("--jsonl-output", default="test/recorded_goals.jsonl")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.odom_topic == "":
        args.odom_topic = None

    rclpy.init()
    node = GoalRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
