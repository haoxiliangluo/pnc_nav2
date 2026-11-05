#!/usr/bin/env python3
"""LIO-SAM 长距离漂移测试：矩形闭环轨迹，比对 LIO-SAM 位姿与 gz 真值。

用法（仿真已启动，见 run_lio_drift.sh）：
    python3 test/lio_drift_test.py --laps 3 --forward 4.0 --side 3.0

输出：
  - 终端表格（每圈误差、最终误差、ATE、回环跳变）
  - JSON + CSV 到 --out 目录（默认 /tmp/lio_drift_<时间戳>）

说明：
  - 真值取 gz `/odom`（DiffDrive 插件的里程计，仿真里等于真值）
  - 估计值取 `/lio_sam/mapping/odometry`（BEST_EFFORT QoS，需注意匹配）
  - 起点：机器人在世界原点、朝向 +x，LIO-SAM 以其启动位置为 map 原点
"""

import argparse
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class DriftTest(Node):
    def __init__(self, out_dir, gt_topic=''):
        super().__init__('lio_drift_test')
        self.gt = None          # /odom（轮速里程计，原地转时会打滑，仅作后备）
        self.gt_tf = None       # gz 世界真位姿（dynamic_pose/info，首选）
        self.lio = None
        self.samples = []
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self._on_gt, 20)
        if gt_topic:
            self.create_subscription(TFMessage, gt_topic, self._on_gt_tf, 20)
        self.create_subscription(Odometry, '/lio_sam/mapping/odometry', self._on_lio, best_effort)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.closure_events = []

    # ---------- 订阅回调 ----------
    def _on_gt(self, msg):
        self.gt = msg

    def _on_gt_tf(self, msg):
        # gz world pose：transforms[0] 是机器人基座
        if msg.transforms:
            self.gt_tf = msg.transforms[0].transform

    def _on_lio(self, msg):
        if self.lio is not None:
            # 检测回环/优化带来的位姿跳变（真值同时刻不可能跳）
            d = math.hypot(msg.pose.pose.position.x - self.lio.pose.pose.position.x,
                           msg.pose.pose.position.y - self.lio.pose.pose.position.y)
            if d > 0.30:
                self.closure_events.append({
                    't': self.get_clock().now().nanoseconds * 1e-9,
                    'jump_m': round(d, 3),
                    'x': round(msg.pose.pose.position.x, 3),
                    'y': round(msg.pose.pose.position.y, 3),
                })
        self.lio = msg

    # ---------- 基础动作 ----------
    def _spin(self, sec=0.05):
        rclpy.spin_once(self, timeout_sec=sec)

    def _wait_ready(self, timeout=60.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._spin(0.1)
            if (self.gt is not None or self.gt_tf is not None) and self.lio is not None:
                return True
        return False

    def _gt_xy_yaw(self):
        if self.gt_tf is not None:
            p = self.gt_tf
            return p.translation.x, p.translation.y, yaw_of(p.rotation)
        p = self.gt.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def _lio_xy_yaw(self):
        p = self.lio.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def drive_distance(self, dist, speed=0.22, tol=0.03, timeout=60.0):
        """沿当前朝向前进 dist（用真值闭环控制距离与朝向）。"""
        x0, y0, yaw_target = self._gt_xy_yaw()
        t0 = time.time()
        cmd = Twist()
        while time.time() - t0 < timeout:
            x, y, yaw = self._gt_xy_yaw()
            traveled = math.hypot(x - x0, y - y0)
            if traveled >= dist - tol:
                break
            err = wrap(yaw_target - yaw)
            cmd.linear.x = max(0.05, speed * min(1.0, (dist - traveled) / 0.4))
            cmd.angular.z = max(-0.8, min(0.8, 2.0 * err))
            self.pub.publish(cmd)
            self._spin(0.02)
            self._sample()
        self.stop()
        return

    def rotate(self, angle, rate=0.5, tol=math.radians(2.0), timeout=20.0):
        """原地转 angle 弧度（正=逆时针）。"""
        _, _, yaw0 = self._gt_xy_yaw()
        target = wrap(yaw0 + angle)
        cmd = Twist()
        t0 = time.time()
        while time.time() - t0 < timeout:
            _, _, yaw = self._gt_xy_yaw()
            err = wrap(target - yaw)
            if abs(err) < tol:
                break
            cmd.angular.z = max(-rate, min(rate, 2.0 * err))
            self.pub.publish(cmd)
            self._spin(0.02)
            self._sample()
        self.stop()

    def stop(self):
        cmd = Twist()
        for _ in range(15):
            self.pub.publish(cmd)
            self._spin(0.02)

    def _sample(self):
        if (self.gt is None and self.gt_tf is None) or self.lio is None:
            return
        gx, gy, gyaw = self._gt_xy_yaw()
        lx, ly, lyaw = self._lio_xy_yaw()
        self.samples.append({
            't': (self.gt.header.stamp.sec + self.gt.header.stamp.nanosec * 1e-9)
                 if self.gt is not None else 0.0,
            'gt_x': gx, 'gt_y': gy, 'gt_yaw': gyaw,
            'lio_x': lx, 'lio_y': ly, 'lio_yaw': lyaw,
            'err': math.hypot(gx - lx, gy - ly),
            'yaw_err': abs(wrap(gyaw - lyaw)),
        })

    # ---------- 报告 ----------
    def report(self, laps, out_dir):
        s = self.samples
        if not s:
            print("没有采样数据")
            return None
        ate = math.sqrt(sum(x['err'] ** 2 for x in s) / len(s))
        last = s[-1]
        summary = {
            'samples': len(s),
            'distance_gt_m': round(sum(math.hypot(b['gt_x'] - a['gt_x'], b['gt_y'] - a['gt_y'])
                                       for a, b in zip(s, s[1:])), 2),
            'ate_rmse_m': round(ate, 3),
            'final_err_m': round(last['err'], 3),
            'final_yaw_err_deg': round(math.degrees(last['yaw_err']), 2),
            'max_err_m': round(max(x['err'] for x in s), 3),
            'loop_closure_jumps': len(self.closure_events),
            'lap_end_errors': self.lap_errors,
        }
        with open(os.path.join(out_dir, 'drift.json'), 'w') as f:
            json.dump({'summary': summary, 'closure_events': self.closure_events}, f, indent=2)
        with open(os.path.join(out_dir, 'drift.csv'), 'w') as f:
            f.write('t,gt_x,gt_y,gt_yaw,lio_x,lio_y,lio_yaw,err,yaw_err\n')
            for x in s:
                f.write(f"{x['t']:.3f},{x['gt_x']:.4f},{x['gt_y']:.4f},{x['gt_yaw']:.4f},"
                        f"{x['lio_x']:.4f},{x['lio_y']:.4f},{x['lio_yaw']:.4f},"
                        f"{x['err']:.4f},{x['yaw_err']:.4f}\n")

        print("\n=== LIO-SAM 漂移测试 ===")
        print(f"采样 {summary['samples']} 帧，真值里程 {summary['distance_gt_m']} m，{laps} 圈")
        print("圈末尾误差 (m):", [round(e, 3) for e in self.lap_errors])
        print(f"ATE RMSE      : {summary['ate_rmse_m']:.3f} m")
        print(f"最大位置误差  : {summary['max_err_m']:.3f} m")
        print(f"最终位置误差  : {summary['final_err_m']:.3f} m")
        print(f"最终航向误差  : {summary['final_yaw_err_deg']:.2f} deg")
        print(f"回环跳变次数  : {summary['loop_closure_jumps']}  {self.closure_events}")
        print(f"结果目录      : {out_dir}")
        return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--laps', type=int, default=3)
    ap.add_argument('--forward', type=float, default=4.0, help='矩形长边 (m)')
    ap.add_argument('--side', type=float, default=3.0, help='矩形短边 (m)')
    ap.add_argument('--speed', type=float, default=0.22)
    ap.add_argument('--out', default=None)
    ap.add_argument('--ready-timeout', type=float, default=60.0)
    ap.add_argument('--gt-topic', default='',
                    help='gz 世界真值话题（如 /world/lio_room/dynamic_pose/info）；留空则用 /odom')
    args = ap.parse_args()

    out_dir = args.out or f"/tmp/lio_drift_{time.strftime('%Y%m%d_%H%M%S')}"
    rclpy.init()
    node = DriftTest(out_dir, gt_topic=args.gt_topic)
    node.lap_errors = []

    if not node._wait_ready(args.ready_timeout):
        print("等待 /odom + /lio_sam/mapping/odometry 超时（检查 LIO-SAM 是否启动、QoS）")
        rclpy.shutdown()
        return 1

    print(f"起点 GT: {node._gt_xy_yaw()}, LIO: {node._lio_xy_yaw()}  轨迹 4x3 矩形 x {args.laps} 圈")
    for lap in range(1, args.laps + 1):
        node.drive_distance(args.forward, speed=args.speed)
        node.rotate(math.pi / 2)
        node.drive_distance(args.side, speed=args.speed)
        node.rotate(math.pi / 2)
        node.drive_distance(args.forward, speed=args.speed)
        node.rotate(math.pi / 2)
        node.drive_distance(args.side, speed=args.speed)
        node.rotate(math.pi / 2)
        # 每圈结束停 1.5s，让 LIO-SAM 处理完队列（含回环检测）
        node.stop()
        t0 = time.time()
        while time.time() - t0 < 1.5:
            node._spin(0.05)
        node._sample()
        node.lap_errors.append(node.samples[-1]['err'])
        print(f"  第 {lap} 圈完成：位置误差 {node.samples[-1]['err']:.3f} m")

    node.report(args.laps, out_dir)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
