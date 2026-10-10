#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cloud_to_base.py —— 把雷达点云从传感器坐标系搬到 base_footprint

为什么需要它：
  pcl_localization_ros2（NDT 那套）**不做外参补偿**，它默认“收到的点云已经在
  base_frame 里”。而 fzsd 的 Mid-360 装在云台上：base_footprint -> gimbal_yaw ->
  front_mid360，平移 (0.045, 0.191, 0.494) 且带 -45° roll。
  不搬的话，NDT 会把“车歪了 45°”当成解，map->odom 直接废掉。

做法：
  用 TF 把 cloud 从它自己的 frame_id 变换到 base_frame（用点云自己的时间戳，
  所以云台转动时也是对的），再重新发出去。

订阅：/livox/lidar          （frame_id: front_mid360，50 Hz）
发布：/livox/lidar_base     （frame_id: base_footprint）

用法：
  正常情况用 launch（见 hdl_loc.launch.py），或者
  python3 cloud_to_base.py --ros-args -p base_frame:=base_footprint

TF 拿不到时（比如 joint_states 没发、front_mid360 不在 TF 树里）：
  -p use_tf:=false -p fallback_xyzrpy:="[0.04541,0.19059,0.4936,-0.7854,0,0]"
  退回成固定外参，保证一定能跑起来。
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros


def rpy_to_matrix(roll, pitch, yaw):
    """rpy(弧度) -> 3x3 旋转矩阵，ZYX 顺序（与 URDF / static_transform_publisher 一致）"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


class CloudToBase(Node):
    def __init__(self):
        super().__init__("cloud_to_base")

        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/lidar_base")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("use_tf", True)
        self.declare_parameter("tf_timeout", 0.05)
        # use_tf:=false 时的固定外参：先旋转再平移，p_base = R*p_sensor + t，单位 m / rad
        self.declare_parameter("fallback_xyzrpy",
                               [0.04541, 0.19059, 0.4936, -0.7854, 0.0, 0.0])
        # 输出限频（0 = 不限）；点云太密想省算力时可设 20.0
        self.declare_parameter("max_rate", 0.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.use_tf = bool(self.get_parameter("use_tf").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.max_rate = float(self.get_parameter("max_rate").value)
        f = list(self.get_parameter("fallback_xyzrpy").value)
        self.fb_t = np.array(f[0:3], dtype=float)
        self.fb_R = rpy_to_matrix(*f[3:6])

        # 订阅用 best_effort：既能收 reliable 也能收 best_effort 的发布者
        sub_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE)
        # 发布用 reliable：best_effort 与 reliable 的订阅者都能收到
        pub_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, pub_qos)
        self.sub = self.create_subscription(
            PointCloud2, self.get_parameter("input_topic").value,
            self.on_cloud, sub_qos)

        self.last_pub_time = None
        self.n_in = 0
        self.n_out = 0
        self.n_fail = 0
        self.warn_count = 0
        self.get_logger().info(
            "cloud_to_base 启动：%s -> %s，目标坐标系 %s，外参来源=%s" % (
                self.get_parameter("input_topic").value,
                self.get_parameter("output_topic").value,
                self.base_frame, "TF" if self.use_tf else "固定参数"))

    # ---- 取变换 --------------------------------------------------------------
    def lookup(self, stamp, child_frame):
        """返回 (R, t)；失败返回 None"""
        if not self.use_tf:
            return self.fb_R, self.fb_t
        if not child_frame:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, child_frame, stamp,
                rclpy.duration.Duration(seconds=self.tf_timeout))
        except Exception:
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n < 1e-6:
            return None
        x, y, z, w = x / n, y / n, z / n, w / n
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        return R, np.array([t.x, t.y, t.z], dtype=float)

    # ---- 取点 ---------------------------------------------------------------
    def read_xyz_i(self, msg):
        """读出 (N,4) 的 x,y,z,intensity float32 数组；intensity 缺失时补 0"""
        try:
            arr = pc2.read_points_numpy(
                msg, field_names=("x", "y", "z", "intensity"), skip_nans=False)
            arr = np.asarray(arr, dtype=np.float32).reshape(-1, 4)
            m = np.isfinite(arr[:, 0]) & np.isfinite(arr[:, 1]) & np.isfinite(arr[:, 2])
            return arr[m]
        except Exception:
            pass
        try:
            arr = pc2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=False)
            arr = np.asarray(arr, dtype=np.float32).reshape(-1, 3)
            m = np.isfinite(arr[:, 0]) & np.isfinite(arr[:, 1]) & np.isfinite(arr[:, 2])
            arr = arr[m]
            out = np.zeros((arr.shape[0], 4), dtype=np.float32)
            out[:, 0:3] = arr
            return out
        except Exception:
            pts = [(float(p[0]), float(p[1]), float(p[2]), 0.0) for p in
                   pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]
            if not pts:
                return np.zeros((0, 4), dtype=np.float32)
            return np.asarray(pts, dtype=np.float32)

    def on_cloud(self, msg: PointCloud2):
        self.n_in += 1

        if self.max_rate > 0.0:
            now_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if (self.last_pub_time is not None and
                    (now_s - self.last_pub_time) < (1.0 / self.max_rate)):
                return
            self.last_pub_time = now_s

        tr = self.lookup(msg.header.stamp, msg.header.frame_id)
        if tr is None:
            self.n_fail += 1
            if self.warn_count < 5:
                self.warn_count += 1
                self.get_logger().warn(
                    "拿不到 %s -> %s 的 TF（第 %d 次失败）。检查 front_mid360 是否在 TF 树里、"
                    "joint_states 是否在发；或加 -p use_tf:=false 用固定外参。"
                    % (self.base_frame, msg.header.frame_id, self.n_fail))
            return
        R, t = tr

        arr = self.read_xyz_i(msg)
        if arr.shape[0] == 0:
            return
        arr[:, 0:3] = (arr[:, 0:3].astype(np.float64) @ R.T + t).astype(np.float32)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.base_frame
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out = pc2.create_cloud(header, fields, arr)
        self.pub.publish(out)
        self.n_out += 1

        if self.n_out % 500 == 0:
            self.get_logger().info(
                "已转发 %d 帧（收到 %d，TF 失败 %d），本帧 %d 点"
                % (self.n_out, self.n_in, self.n_fail, arr.shape[0]))


def main():
    rclpy.init()
    node = CloudToBase()
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
