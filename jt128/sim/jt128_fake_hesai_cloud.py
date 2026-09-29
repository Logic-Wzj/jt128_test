#!/usr/bin/env python3
"""发布一份**禾赛 JT128 布局**的合成点云，用于在没有雷达时回归测试下游适配层。

字段与真雷达一致（`src/manager/source_driver_ros2.hpp:274-279`）：
    x,y,z,intensity (float32) + ring (uint16) + timestamp (float64)   → point_step 26
其中 `timestamp` 刻意用**秒**（雷达上电累计秒，正是实测到的单位：一帧跨度 0.1 s），
用来验证 jt128_livox_compat 的 `timestamp_scale`（默认 ×1e9 秒→纳秒）有没有生效。

    python3 jt128_fake_hesai_cloud.py                      # 默认 10 Hz / 2 万点 / 发到 /lidar_points_test
    python3 jt128_fake_hesai_cloud.py --ros-args -p topic:=/lidar_points -p points_per_frame:=230400
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField

# 与禾赛驱动一致的字段布局（align=False 时 numpy 的偏移正好是 0/4/8/12/16/18）
DT = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('intensity', '<f4'), ('ring', '<u2'), ('timestamp', '<f8'),
])
FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
    PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1),
]


class FakeHesaiCloud(Node):
    def __init__(self):
        super().__init__('jt128_fake_hesai_cloud')
        self.declare_parameter('topic', '/lidar_points_test')
        self.declare_parameter('frame_id', 'front_jt128')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('points_per_frame', 20000)
        self.declare_parameter('rings', 128)          # JT128 是 128 线
        self.declare_parameter('ts_base_sec', 1000.0)  # 模拟"雷达上电 1000 秒"
        self.declare_parameter('ts_span_sec', 0.1)     # 一帧跨度：10 Hz → 0.1 s

        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.n = int(self.get_parameter('points_per_frame').value)
        self.rings = int(self.get_parameter('rings').value)
        self.ts_base = float(self.get_parameter('ts_base_sec').value)
        self.ts_span = float(self.get_parameter('ts_span_sec').value)
        rate = float(self.get_parameter('rate_hz').value)

        # 预先算好一帧的点（球面上的假点云，够下游统计用）
        ang_h = np.linspace(0, 2 * np.pi, self.n, endpoint=False)
        ring = (np.arange(self.n) % self.rings).astype(np.uint16)
        elev = -0.4 + 1.2 * (ring.astype(np.float64) / max(1, self.rings - 1))
        r = 2.0 + 3.0 * np.sin(ang_h * 3.0) ** 2          # 2~5 m，形状无所谓
        self.pts = np.zeros(self.n, dtype=DT)
        self.pts['x'] = (r * np.cos(elev) * np.cos(ang_h)).astype(np.float32)
        self.pts['y'] = (r * np.cos(elev) * np.sin(ang_h)).astype(np.float32)
        self.pts['z'] = (r * np.sin(elev)).astype(np.float32)
        self.pts['intensity'] = np.linspace(0, 255, self.n).astype(np.float32)
        self.pts['ring'] = ring
        # 逐点时间戳：**秒**，线性递增覆盖一帧跨度（这是本工具的关键点）
        self.pts['timestamp'] = self.ts_base + np.linspace(0.0, self.ts_span, self.n)

        qos = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(PointCloud2, self.topic, qos)
        self.create_timer(1.0 / rate, self.tick)
        self.k = 0
        self.get_logger().info(
            f'合成禾赛布局点云：{self.topic}  frame={self.frame_id}  {self.n} 点/帧  '
            f'point_step={DT.itemsize}  timestamp={self.ts_base}~{self.ts_base + self.ts_span} **秒**')

    def tick(self):
        self.k += 1
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = self.n
        msg.fields = FIELDS
        msg.is_bigendian = False
        msg.point_step = DT.itemsize
        msg.row_step = msg.width * msg.point_step
        msg.is_dense = False
        msg.data = self.pts.tobytes()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeHesaiCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
