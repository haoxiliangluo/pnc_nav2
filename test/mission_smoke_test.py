#!/usr/bin/env python3
"""任务层冒烟：监听 /mission_status，等到终态并断言是否 SUCCEEDED。

用法：
  python3 test/mission_smoke_test.py [timeout_s]
退出码：0 = SUCCEEDED；1 = 其它终态或超时
"""
import sys
import time

import rclpy
from rclpy.node import Node

from pnc_nav_interfaces.msg import MissionStatus


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    rclpy.init()
    node = Node('mission_smoke_probe')
    state = {'last': None}

    def cb(msg):
        key = (msg.state, msg.current_index, msg.retries_done)
        if key != state['last']:
            state['last'] = key
            print(f"[mission] state={msg.state} idx={msg.current_index}/{msg.total} "
                  f"retries={msg.retries_done} msg='{msg.message}'", flush=True)

    node.create_subscription(MissionStatus, '/mission_status', cb, 10)
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if state['last'] and state['last'][0] in (3, 4, 5):
            break

    ok = bool(state['last'] and state['last'][0] == 3)
    print(f"[mission] final={state['last']} -> {'SUCCEEDED' if ok else 'FAILED/TIMEOUT'}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
