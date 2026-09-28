#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
代价地图占据统计 —— 用来判断"点云/激光有没有真的进 costmap"

用法：
    python3 costmap_occ.py                                  # 统计 local_costmap
    python3 costmap_occ.py --topic /global_costmap/costmap
    python3 costmap_occ.py --save /tmp/occ1.npy             # 存成 npy 便于对比
    python3 costmap_occ.py --diff /tmp/occ1.npy             # 和之前存的对比，看新增/消失的格子

判读：
    占据格数在"起点云 / 停点云"之间**明显变化**，才说明障碍层真的在吃这份数据。
    完全不变（尤其两次读数一模一样）通常意味着：代价地图根本没在更新
    （典型原因：robot_base_frame 与 odom 不在同一棵 TF 树里，nav2 每帧都超时丢弃）。
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import OccupancyGrid


class Occ(Node):
    def __init__(self, topic, use_sim_time):
        super().__init__('costmap_occ')
        self.topic = topic
        self.msg = None
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, topic, self._cb, q)

    def _cb(self, m):
        self.msg = m

    def grab(self, timeout=15.0):
        t0 = time.time()
        while self.msg is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.msg


def summarize(tag, m):
    g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    occ = int((g >= 50).sum())
    free = int(((g >= 0) & (g < 50)).sum())
    unk = int((g < 0).sum())
    print(f"[{tag}] {m.info.width}x{m.info.height} @{m.info.resolution:.2f}m "
          f"原点({m.info.origin.position.x:.1f},{m.info.origin.position.y:.1f}) "
          f"stamp={m.header.stamp.sec}.{m.header.stamp.nanosec // 10**6:03d} | "
          f"占据 {occ} | 空闲 {free} | 未知 {unk}")
    return g, occ


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/local_costmap/costmap')
    ap.add_argument('--save', default=None, help='把栅格存成 .npy')
    ap.add_argument('--diff', default=None, help='与此前存的 .npy 对比')
    ap.add_argument('--timeout', type=float, default=15.0)
    ap.add_argument('--use-sim-time', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    n = Occ(a.topic, a.use_sim_time)
    if a.use_sim_time:
        n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    m = n.grab(a.timeout)
    if m is None:
        print(f"❌ {a.timeout:.0f} 秒内没收到 {a.topic}（代价地图没在发布？）")
        n.destroy_node(); rclpy.shutdown(); return 1
    g, occ = summarize('now', m)

    if a.save:
        np.save(a.save, g)
        print(f"   已存盘: {a.save}")

    if a.diff and os.path.isfile(a.diff):
        old = np.load(a.diff)
        if old.shape != g.shape:
            print(f"   ⚠️ 尺寸不同（旧 {old.shape} vs 新 {g.shape}），无法逐格对比")
        else:
            new_occ = ((old < 50) & (g >= 50)).sum()
            gone = ((old >= 50) & (g < 50)).sum()
            print(f"   与 {a.diff} 对比：新增占据 {new_occ} 格，消失 {gone} 格")
            if new_occ == 0 and gone == 0:
                print("   → 完全没变化。若两轮之间你切换了点云/激光源，说明代价地图没吃到它，")
                print("     先去仿真终端看有没有 'Timed out waiting for transform ... to odom' 之类的报错。")
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
