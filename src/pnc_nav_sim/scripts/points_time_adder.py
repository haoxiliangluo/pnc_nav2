#!/usr/bin/env python3
"""给 gz 的 PointCloud2 补逐点时间戳（LIO-SAM 的 deskew 需要 `time` 字段）。

gz 的 gpu_lidar 只输出 x/y/z/intensity(+ring)，没有逐点 time，导致 LIO-SAM 打印
"Point cloud timestamp not available, deskew function disabled, system will drift
significantly!" 并且确实会漂。这里按方位角线性插值合成 time（一圈 = scan_period）。

订阅 /points → 发布 /points_time（字段：x y z intensity ring time，time 为 float32 秒）。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class PointsTimeAdder(Node):
    def __init__(self):
        super().__init__('points_time_adder')
        self.declare_parameter('input_topic', 'points')
        self.declare_parameter('output_topic', 'points_time')
        self.declare_parameter('scan_period', 0.1)
        self.scan_period = float(self.get_parameter('scan_period').value)
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(PointCloud2, out, qos)
        self.create_subscription(PointCloud2, inp, self.cb, qos)
        self.fields_out = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=24, datatype=PointField.UINT16, count=1),
            PointField(name='time', offset=28, datatype=PointField.FLOAT32, count=1),
        ]
        self.n = 0
        self.get_logger().info(f'time 注入：{inp} -> {out}（scan_period={self.scan_period}s）')

    def cb(self, msg: PointCloud2):
        n = msg.width * msg.height
        if n == 0:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        step = msg.point_step
        try:
            dt_in = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring'],
                              'formats': ['<f4', '<f4', '<f4', '<f4', '<u2'],
                              'offsets': [0, 4, 8, 16, 24], 'itemsize': step})
            pts = raw.reshape(n, step).view(dt_in).reshape(n)
        except Exception as e:  # 字段布局不同就直说
            self.get_logger().error(f'点云布局不支持: {e}', once=True)
            return

        out = np.zeros(n, dtype=np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                                          'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                                          'offsets': [0, 4, 8, 16, 24, 28], 'itemsize': 32}))
        out['x'] = pts['x']; out['y'] = pts['y']; out['z'] = pts['z']
        out['intensity'] = pts['intensity']; out['ring'] = pts['ring']

        az = np.arctan2(out['y'], out['x'])
        az = np.where(az < 0, az + 2 * np.pi, az)
        out['time'] = (az / (2 * np.pi) * self.scan_period).astype(np.float32)

        m = PointCloud2()
        m.header = msg.header
        m.height = msg.height
        m.width = msg.width
        m.fields = self.fields_out
        m.is_bigendian = False
        m.point_step = 32
        m.row_step = 32 * msg.width
        m.is_dense = msg.is_dense
        m.data = out.tobytes()
        self.pub.publish(m)
        self.n += 1
        if self.n % 100 == 0:
            self.get_logger().info(f'已转发 {self.n} 帧')


def main():
    rclpy.init()
    node = PointsTimeAdder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
