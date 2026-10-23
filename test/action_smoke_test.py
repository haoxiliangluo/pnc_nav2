#!/usr/bin/env python3
"""pnc_nav2 NavigateTo3DPose action 冒烟测试。

用法:
  python3 test_action.py feedback   # 发可达目标，观察 feedback，到达后返回
  python3 test_action.py cancel     # 发远目标，2s 后取消
  python3 test_action.py unreachable# 发地图外目标，期望失败 + 错误码
"""
import sys
import time
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from pnc_nav_interfaces.action import NavigateTo3DPose


class Tester(Node):
    def __init__(self):
        super().__init__('navigate_action_tester')
        self.client = ActionClient(self, NavigateTo3DPose, 'navigate_to_3d_pose')
        self.last_feedback = None
        self._lock = threading.Lock()

    def feedback_cb(self, msg):
        fb = msg.feedback
        with self._lock:
            self.last_feedback = fb
        print(
            f"[feedback] state={fb.current_state:<10} "
            f"dist={fb.distance_remaining:.2f}m eta={fb.eta_seconds:.1f}s "
            f"recoveries={fb.number_of_recoveries} "
            f"pose=({fb.current_pose.pose.position.x:.2f},{fb.current_pose.pose.position.y:.2f})",
            flush=True,
        )

    def make_goal(self, x, y):
        goal = NavigateTo3DPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = float(x)
        goal.goal.pose.position.y = float(y)
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = ''
        goal.max_planning_time = 0.0
        return goal


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'feedback'
    rclpy.init()
    node = Tester()
    node.get_logger().info('waiting for action server ...')
    if not node.client.wait_for_server(timeout_sec=10.0):
        print('FAIL: action server 不可用')
        return 1

    if mode == 'unreachable':
        x, y = 999.0, 999.0
    elif mode == 'cancel':
        x, y = 5.0, 0.0
    else:
        x, y = 2.0, 0.0

    # 可选：命令行覆盖目标点，例如 `feedback 4.0 0.0`
    if len(sys.argv) >= 4:
        x, y = float(sys.argv[2]), float(sys.argv[3])

    print(f'== send goal ({x}, {y}) mode={mode} ==', flush=True)
    send_future = node.client.send_goal_async(
        node.make_goal(x, y), feedback_callback=node.feedback_cb)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        print('FAIL: goal 被拒绝')
        return 1
    print('goal accepted', flush=True)

    if mode == 'cancel':
        time.sleep(2.0)
        print('== cancel goal ==', flush=True)
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
        print('cancel response:', cancel_future.result(), flush=True)

    result_future = goal_handle.get_result_async()
    deadline = time.time() + (60.0 if mode != 'cancel' else 15.0)
    while time.time() < deadline and not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.2)

    if not result_future.done():
        print('FAIL: 等 result 超时')
        return 1

    result = result_future.result().result
    print('== result ==')
    print(f'  success        = {result.success}')
    print(f'  error_code     = {result.error_code}')
    print(f'  total_time     = {result.total_time:.2f} s')
    print(f'  distance       = {result.distance_traveled:.2f} m')
    print(f'  message        = {result.message}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
