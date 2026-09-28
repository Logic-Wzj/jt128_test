#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JT128 → Gazebo 仿真 HIL 注入节点（硬件在环）

作用：把真实雷达的点云搬进仿真里的导航栈。它做三件必须做的事：

  1. **改写 frame_id**：真雷达发的是 front_jt128（真实机器人 URDF 里的帧），
     而仿真 TF 树里只有 front_mid360（SDF 里的雷达 link）。不改写的话
     costmap 会报 "frame does not exist"，点云直接丢掉。
  2. **改写时间戳为仿真时钟**：真雷达的时间戳是绝对 UTC（内部时钟/GNSS/PTP），
     而仿真是 use_sim_time=True（Gazebo /clock）。不改成仿真时间，costmap 做
     TF 变换时会报 "Lookup would require extrapolation into the future/past"，
     点云同样用不了。这是 HIL 最容易踩的坑。
  3. **可选抽稀**：JT128 单回波 115200 点/帧、双回波 230400 点/帧，是仿真原
     本那颗 Mid-360 的数倍。costmap 每帧都做 TF + 栅格化，点太多会拖慢 nav2，
     用 stride 或 max_points 抽稀。

话题默认：/lidar_points  →  /jt128/points

用法：
    ros2 run 不方便（脚本没打包），直接 ros2 launch 本目录的 launch 文件，
    或手动：python3 jt128_sim_inject.py --ros-args -p target_frame:=front_mid360
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class Jt128SimInject(Node):
    def __init__(self):
        super().__init__('jt128_sim_inject')

        self.declare_parameter('in_topic', '/lidar_points')
        self.declare_parameter('out_topic', '/jt128/points')
        self.declare_parameter('target_frame', 'front_mid360')
        self.declare_parameter('stride', 1)          # 每 N 个点保留 1 个，1=不抽稀
        self.declare_parameter('max_points', 0)      # >0 时按帧均匀抽到该点数，0=不限
        self.declare_parameter('stamp_from_sim_time', True)
        self.declare_parameter('log_interval_sec', 2.0)
        # 传感器流用 best-effort：1~2 MB/帧的大点云配 RELIABLE 会被慢订阅者反压
        # （实测被 `ros2 topic hz` 这种可靠 CLI 订阅者拖到 2.5 Hz）；best-effort 下慢
        # 订阅者只是丢帧，不拖慢发布端。nav2 costmap 也是按 SensorDataQoS 订阅的。
        self.declare_parameter('qos_reliable', False)
        # 注意：use_sim_time 是 rclpy 内置参数，不能重复声明（会 ParameterAlreadyDeclared）

        p = self.get_parameter
        self.in_topic = p('in_topic').value
        self.out_topic = p('out_topic').value
        self.target_frame = p('target_frame').value
        self.stride = max(1, int(p('stride').value))
        self.max_points = max(0, int(p('max_points').value))
        self.stamp_from_sim_time = bool(p('stamp_from_sim_time').value)
        self.log_interval = float(p('log_interval_sec').value)
        reliable = bool(p('qos_reliable').value)

        # 订阅：真雷达驱动默认 reliable 发布，best-effort 订阅兼容
        sub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        pub_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE if reliable
                             else ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(PointCloud2, self.out_topic, pub_qos)
        self.sub = self.create_subscription(PointCloud2, self.in_topic, self.cb, sub_qos)

        self.n_in = self.n_out = 0
        self.pts_in = self.pts_out = 0
        self.t_last_log = self.get_clock().now()
        self.n_in_at_log = 0

        self.get_logger().info(
            f"HIL 注入：{self.in_topic} -> {self.out_topic} | frame -> {self.target_frame} | "
            f"stride={self.stride} max_points={self.max_points or '不限'} | "
            f"时间戳={'仿真时钟' if self.stamp_from_sim_time else '原始'}")

    def cb(self, msg: PointCloud2):
        n = msg.width * msg.height
        if n == 0 or msg.point_step == 0:
            return
        self.n_in += 1
        self.pts_in += n

        # 计算抽样步长
        step = self.stride
        if self.max_points and n > self.max_points:
            step = max(step, int(math.ceil(n / float(self.max_points))))

        out = PointCloud2()
        out.header.frame_id = self.target_frame
        out.header.stamp = (self.get_clock().now().to_msg()
                            if self.stamp_from_sim_time else msg.header.stamp)
        out.fields = msg.fields          # 字段偏移不变（整条记录搬运）
        out.point_step = msg.point_step
        out.height = 1
        out.is_dense = msg.is_dense

        if step == 1:
            out.width = n
            out.data = msg.data           # 零拷贝共享
        else:
            # 按记录抽样：把 data 看成 (n, point_step) 的字节矩阵，取每 step 行
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
            sel = arr[::step]
            out.width = int(sel.shape[0])
            out.data = sel.tobytes()
        out.row_step = out.width * out.point_step

        self.n_out += 1
        self.pts_out += out.width
        self.pub.publish(out)

        now = self.get_clock().now()
        dt = (now - self.t_last_log).nanoseconds / 1e9
        if dt >= self.log_interval:
            frames = self.n_in - self.n_in_at_log
            self.get_logger().info(
                f"{frames / dt:.1f} 帧/s | 入 {n} 点 -> 出 {out.width} 点 | "
                f"累计 {self.n_out} 帧 / {self.pts_out / 1e6:.1f} M 点")
            self.t_last_log = now
            self.n_in_at_log = self.n_in


def main():
    rclpy.init()
    node = Jt128SimInject()
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
