#!/usr/bin/env python3
"""实验运行器：按 yaml 配置跑多次导航任务，输出 JSON + CSV。

用法：
  python3 scripts/experiment_runner.py experiments/planner_comparison.yaml
  # 结果：experiments/results/<时间戳>/{runs.json, summary.csv}

配置字段：
  name / profile / goal([x,y]) / runs / timeout / cases[{name, params_file}]
"""
import csv
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

from pnc_nav_interfaces.action import NavigateTo3DPose

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOMAIN = os.environ.setdefault('ROS_DOMAIN_ID', '43')
PARTITION = os.environ.setdefault('GZ_PARTITION', 'pnc_nav_test')


def kill_pnc_procs():
    """只清本项目 install 目录下的进程"""
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            cmd = open(f'/proc/{pid}/cmdline', 'rb').read().decode(errors='ignore').replace('\0', ' ')
        except OSError:
            continue
        if 'pnc_nav2/install' in cmd and 'grep' not in cmd:
            try:
                os.kill(int(pid), signal.SIGKILL)
            except OSError:
                pass
    time.sleep(1.5)


def shell_ok(cmd, timeout=6):
    try:
        out = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return out.returncode == 0
    except subprocess.TimeoutExpired:
        return False


def spawn_ok(name, retries=3):
    """spawn 后确认模型真的在 gz 世界里（否则消融实验会结论失真）"""
    for _ in range(retries):
        time.sleep(1.0)
        if shell_ok(f"gz model --list 2>/dev/null | grep -q '{name}'", timeout=8):
            return True
        time.sleep(1.0)
    return False


def wait_ready(timeout=90):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if shell_ok('ros2 action list 2>/dev/null | grep -q navigate_to_3d_pose'):
            if shell_ok('ros2 topic list 2>/dev/null | grep -qx /odom'):
                time.sleep(3)
                return True
        time.sleep(1)
    return False


class Probe(Node):
    def __init__(self):
        super().__init__('experiment_probe')
        self.client = ActionClient(self, NavigateTo3DPose, 'navigate_to_3d_pose')
        self.last_pose = None
        self.last_dist = None
        self.create_subscription(Odometry, '/odom', self.cb_odom, 20)

    def cb_odom(self, msg):
        p = msg.pose.pose.position
        self.last_pose = (p.x, p.y)

    def send_goal(self, x, y, timeout):
        if not self.client.wait_for_server(timeout_sec=10.0):
            return {'success': False, 'error_code': -1, 'message': 'no action server',
                    'total_time': 0.0, 'distance_traveled': 0.0}
        goal = NavigateTo3DPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = float(x)
        goal.goal.pose.position.y = float(y)
        goal.goal.pose.orientation.w = 1.0

        start = time.time()
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return {'success': False, 'error_code': -1, 'message': 'goal rejected',
                    'total_time': 0.0, 'distance_traveled': 0.0}

        res_fut = handle.get_result_async()
        while time.time() - start < timeout and not res_fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not res_fut.done():
            return {'success': False, 'error_code': -2, 'message': 'timeout',
                    'total_time': time.time() - start, 'distance_traveled': 0.0}
        r = res_fut.result().result
        return {'success': bool(r.success), 'error_code': int(r.error_code),
                'message': r.message, 'total_time': float(r.total_time),
                'distance_traveled': float(r.distance_traveled),
                'final_pose': self.last_pose}


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    cfg_path = sys.argv[1]
    cfg = yaml.safe_load(open(cfg_path))
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(ROOT, 'experiments', 'results', f'{stamp}_{cfg["name"]}')
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    probe = Probe()

    rows = []
    for case in cfg['cases']:
        case_profile = case.get('profile') or cfg['profile']
        params = case.get('params_file') or ''
        params_arg = f'params_file:={os.path.join(ROOT, params)}' if params else ''
        for run in range(1, int(cfg['runs']) + 1):
            print(f"\n=== {case['name']} run {run}/{cfg['runs']} ===", flush=True)
            kill_pnc_procs()
            cmd = (f"ros2 launch pnc_nav_bringup bringup.launch.py profile:={case_profile} "
                   f"use_gui:=false use_rviz:=false {params_arg} "
                   f"> /tmp/pnc_exp_{case['name']}_{run}.log 2>&1 &")
            subprocess.Popen(cmd, shell=True)
            if not wait_ready(90):
                result = {'success': False, 'error_code': -3, 'message': 'bringup timeout',
                          'total_time': 0.0, 'distance_traveled': 0.0, 'final_pose': None}
            else:
                setup_cmd = case.get('setup_cmd') or ''
                setup_ok = True
                if setup_cmd:
                    print(f"  setup: {setup_cmd}", flush=True)
                    subprocess.run(setup_cmd, shell=True, capture_output=True)
                    setup_name = case.get('setup_model_name') or 'exp_box'
                    setup_ok = spawn_ok(setup_name)
                    print(f"  setup verified: {setup_ok}", flush=True)
                    time.sleep(float(case.get('setup_wait', 1.5)))
                if not setup_ok:
                    result = {'success': False, 'error_code': -4, 'message': 'setup failed',
                              'total_time': 0.0, 'distance_traveled': 0.0, 'final_pose': None}
                    print("  -> setup failed, 跳过本次", flush=True)
                    rows.append({'case': case['name'], 'run': run, **result})
                    kill_pnc_procs()
                    continue
                result = probe.send_goal(cfg['goal'][0], cfg['goal'][1],
                                         float(cfg['timeout']))
            print(f"  -> success={result['success']} code={result['error_code']} "
                  f"time={result['total_time']:.1f}s dist={result['distance_traveled']:.2f}m",
                  flush=True)
            rows.append({'case': case['name'], 'run': run, **result})
            kill_pnc_procs()

    probe.destroy_node()
    rclpy.shutdown()

    json.dump({'config': cfg, 'rows': rows},
              open(os.path.join(out_dir, 'runs.json'), 'w'), indent=2, ensure_ascii=False)

    csv_path = os.path.join(out_dir, 'summary.csv')
    keys = ['case', 'run', 'success', 'error_code', 'total_time', 'distance_traveled', 'message']
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in keys})

    print(f"\n结果已保存：{out_dir}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
