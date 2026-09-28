#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JT128 假点云源 —— 没有雷达时先把 HIL 链路跑通（干跑）

发布与禾赛驱动同结构的 PointCloud2（x/y/z/intensity，float32，height=1），
在雷达前方合成一个"房间 + 柱子"的点云，用来验证：
    /jt128/points 话题 → 仿真 local_costmap → MPPI 是否真的吃到了点云

字段刻意做得比真雷达简单（真雷达还带 ring/timestamp），因为 costmap 只用 x/y/z；
点数默认 115200（对齐 JT128 单回波 10 Hz 的真实负载），可用 points_per_frame 调。

用法：
    python3 jt128_fake_cloud.py --ros-args -p points_per_frame:=20000 -p use_sim_time:=true
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField


class FakeJt128Cloud(Node):
    def __init__(self):
        super().__init__('jt128_fake_cloud')

        self.declare_parameter('topic', '/jt128/points')
        self.declare_parameter('frame_id', 'front_mid360')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('points_per_frame', 115200)
        self.declare_parameter('room_size', 3.0)       # 四周墙距雷达 ±room_size (m)
        # 把合成点云整体搬到指定位置。**发到 costmap 全局系时必须用**：
        # 例：frame_id:=odom origin_x:=-5.0 origin_y:=3.0 origin_z:=0.2
        # （-5,3 是本机 Gazebo 仿真里 red 车在 odom 系的位置，见 gz_world.yaml）
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('origin_z', 0.0)
        self.declare_parameter('wall_height', 1.5)
        self.declare_parameter('pillar_x', 1.5)        # 正前方柱子
        self.declare_parameter('pillar_radius', 0.2)
        self.declare_parameter('noise_std', 0.01)
        # 注意：use_sim_time 是 rclpy 内置参数，不能重复声明（会 ParameterAlreadyDeclared）

        p = self.get_parameter
        self.topic = p('topic').value
        self.frame_id = p('frame_id').value
        self.rate = float(p('rate_hz').value)
        self.n = max(100, int(p('points_per_frame').value))
        self.room = float(p('room_size').value)
        self.wall_h = float(p('wall_height').value)
        self.px = float(p('pillar_x').value)
        self.pr = float(p('pillar_radius').value)
        self.noise = float(p('noise_std').value)
        self.ox = float(p('origin_x').value)
        self.oy = float(p('origin_y').value)
        self.oz = float(p('origin_z').value)

        # best-effort：大点云 + 慢订阅者不会反压发布端（详见 jt128_sim_inject.py 注释）
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, self.topic, qos)
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.pts = self._make_room()
        self._n = 0
        self._t0 = None
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f"假点云：{self.topic} | frame={self.frame_id} | {self.rate} Hz | "
            f"{self.pts.shape[0]} 点/帧 | 房间 ±{self.room} m + 正前方 {self.px} m 柱子")

    def _make_room(self):
        """合成点：四周墙(占 70%) + 正前方柱子(20%) + 地面若干(10%)"""
        rng = np.random.default_rng(42)
        pts = []
        n_wall = int(self.n * 0.7)
        n_pillar = int(self.n * 0.2)
        n_floor = self.n - n_wall - n_pillar

        # 四周墙：随机选一面，随机高度/位置
        side = rng.integers(0, 4, n_wall)
        t = rng.uniform(-self.room, self.room, n_wall)
        h = rng.uniform(0.0, self.wall_h, n_wall)
        x = np.where(side == 0, self.room, np.where(side == 1, -self.room, t))
        y = np.where(side == 2, self.room, np.where(side == 3, -self.room, t))
        pts.append(np.stack([x, y, h], axis=1))

        # 正前方柱子
        ang = rng.uniform(0, 2 * np.pi, n_pillar)
        h2 = rng.uniform(0.0, self.wall_h, n_pillar)
        pts.append(np.stack([self.px + self.pr * np.cos(ang),
                             self.pr * np.sin(ang), h2], axis=1))

        # 地面
        pts.append(np.stack([rng.uniform(0.3, self.room, n_floor),
                             rng.uniform(-self.room, self.room, n_floor),
                             np.zeros(n_floor)], axis=1))

        pts = np.concatenate(pts, axis=0).astype(np.float32)
        pts += np.array([self.ox, self.oy, self.oz], dtype=np.float32)
        if self.noise > 0:
            pts += rng.normal(0, self.noise, pts.shape).astype(np.float32)
        return pts

    def tick(self):
        cloud = np.zeros((self.pts.shape[0], 4), dtype=np.float32)
        cloud[:, :3] = self.pts
        cloud[:, 3] = 100.0                     # intensity

        msg = PointCloud2()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = self.pts.shape[0]
        msg.fields = self.fields
        msg.point_step = 16
        msg.row_step = msg.width * msg.point_step
        msg.is_dense = True
        t0 = time.time()
        msg.data = cloud.tobytes()
        self.pub.publish(msg)
        dt_pub = (time.time() - t0) * 1000

        # 自报速率：用来判断"慢的是发布侧还是订阅侧"
        self._n += 1
        if self._t0 is None:
            self._t0 = time.time()
        else:
            el = time.time() - self._t0
            if el >= 2.0:
                self.get_logger().info(
                    f"自身发布 {self._n / el:.1f} Hz | publish {dt_pub:.1f} ms | "
                    f"{cloud.nbytes / 1e6:.2f} MB/帧")
                self._n = 0
                self._t0 = time.time()


def main():
    rclpy.init()
    node = FakeJt128Cloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
