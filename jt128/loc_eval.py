#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
loc_eval.py —— 重定位方法对比打分器（GICP vs NDT，或任何发 map->odom 的方案）

原理：
  两种方法对外都只做一件事：广播 map->odom（+ LIO 在发 odom->base）。
  所以"定位好不好"可以从 map->odom 的行为直接量化：
    * 更新率   —— 多久给一次修正（丢定位时直接断流）
    * 跳变量   —— 相邻两次修正差多少（跳变 = 定位抽了）
    * 机器人位姿的地图系运动是否连续（>1.5 m / 0.3 s 视为瞬移）
    * 断流时长 —— 超过 1 s 没有修正 = 可能跟丢

用法：
  跑一段路的同时：
      python3 ~/jt128/loc_eval.py --tag gicp --duration 120
      python3 ~/jt128/loc_eval.py --tag ndt  --duration 120
  跑完对比两份 CSV：
      python3 ~/jt128/loc_eval.py --compare ~/jt128_loc_eval_gicp_*.csv ~/jt128_loc_eval_ndt_*.csv

参数：
  --tag        标签，决定 CSV 文件名
  --duration   采样时长（秒），0 = 一直跑到 Ctrl-C
  --out        输出目录（默认 ~）
  --map-frame / --odom-frame / --base-frame  坐标系名（默认 map/odom/base_footprint）
  --jump       判为"跳变"的阈值（米，默认 0.3）
"""
import argparse
import csv
import glob
import math
import os
import sys
import time
from datetime import datetime

# ROS 相关 import 放在 try 里：--compare 模式只读 CSV，
# 没 source ROS 环境时也要能用（只对比数据，不采集）
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                           DurabilityPolicy)
    from tf2_msgs.msg import TFMessage
    _ROS_OK = True
except ImportError:            # 没 source ROS
    _ROS_OK = False

    class Node:                # 占位基类，只为了让 --compare 模式能 import
        pass


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compose(t1, t2):
    """t = (x,y,z,yaw)；返回 t1 ∘ t2（t1 在外层）"""
    x1, y1, z1, y1aw = t1
    x2, y2, z2, y2aw = t2
    c, s = math.cos(y1aw), math.sin(y1aw)
    return (x1 + c * x2 - s * y2, y1 + s * x2 + c * y2, z1 + z2, y1aw + y2aw)


def yaw_diff(a, b):
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return abs(d)


class LocEval(Node):
    def __init__(self, args):
        super().__init__("loc_eval")
        self.a = args
        self.q = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST,
                            durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(TFMessage, "/tf", self.on_tf, self.q)

        self.map_odom = None          # (x,y,z,yaw) 最近一次修正
        self.map_odom_t = None        # 收到它的时间
        self.odom_base = None
        self.odom_base_stamp = None

        self.events = []              # (t, kind, value)
        self.n_map_odom = 0
        self.last_map_odom = None
        self.last_robot = None
        self.last_robot_t = None
        self.max_gap = 0.0
        self.robot_path = 0.0
        self.t0 = time.time()
        self.first_map_odom = None
        self.warned_no_tf = False

    # ---- TF 回调 ------------------------------------------------------------
    def on_tf(self, msg):
        now = time.time()
        for tr in msg.transforms:
            f, c = tr.header.frame_id, tr.child_frame_id
            q = tr.transform.rotation
            t = tr.transform.translation
            pose = (t.x, t.y, t.z, quat_to_yaw(q.x, q.y, q.z, q.w))

            if f == self.a.map_frame and c == self.a.odom_frame:
                self.n_map_odom += 1
                if self.map_odom is not None:
                    d = math.hypot(pose[0] - self.map_odom[0], pose[1] - self.map_odom[1])
                    dy = yaw_diff(pose[3], self.map_odom[3])
                    self.events.append((now, "map_odom_update", d))
                    if d >= self.a.jump or dy >= math.radians(10.0):
                        self.events.append((now, "JUMP", max(d, dy)))
                if self.first_map_odom is None:
                    self.first_map_odom = pose
                if self.map_odom_t is not None:
                    self.max_gap = max(self.max_gap, now - self.map_odom_t)
                self.map_odom = pose
                self.map_odom_t = now

            elif f == self.a.odom_frame and c == self.a.base_frame:
                self.odom_base = pose
                self.odom_base_stamp = now
                if self.map_odom is None:
                    continue
                robot = compose(self.map_odom, pose)   # map -> base
                if self.last_robot is not None and self.last_robot_t is not None:
                    dt = now - self.last_robot_t
                    d = math.hypot(robot[0] - self.last_robot[0],
                                   robot[1] - self.last_robot[1])
                    self.robot_path += d
                    if dt > 1e-3 and d / dt > 5.0:    # >5 m/s 视为瞬移（麦轮车不可能）
                        self.events.append((now, "ROBOT_TELEPORT", d))
                self.last_robot = robot
                self.last_robot_t = now

    # ---- 周期性打印 ---------------------------------------------------------
    def report(self, final=False):
        el = time.time() - self.t0
        rate = self.n_map_odom / el if el > 0 else 0.0
        jumps = [v for _, k, v in self.events if k == "JUMP"]
        tele = [v for _, k, v in self.events if k == "ROBOT_TELEPORT"]
        gap = (time.time() - self.map_odom_t) if self.map_odom_t else float("nan")
        line = (f"[{el:6.1f}s] map->odom {self.n_map_odom} 次 ({rate:.1f} Hz)"
                f" | 最近一次 {gap:.2f}s 前"
                f" | 跳变 {len(jumps)} 次")
        if jumps:
            line += f" (最大 {max(jumps):.2f} m)"
        print(line, flush=True)
        if tele:
            print(f"         机器人瞬移 {len(tele)} 次 (最大 {max(tele):.2f} m)",
                  flush=True)
        if final:
            # 收尾时把"最后一次更新到现在"也算进最长断流
            if self.map_odom_t:
                self.max_gap = max(self.max_gap, time.time() - self.map_odom_t)
            print(f"  走过的路程（地图系累计）: {self.robot_path:.1f} m")
            print(f"  最长断流: {self.max_gap:.2f} s")

    def dump(self, path):
        jumps = [v for _, k, v in self.events if k == "JUMP"]
        tele = [v for _, k, v in self.events if k == "ROBOT_TELEPORT"]
        el = time.time() - self.t0
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["tag", self.a.tag])
            w.writerow(["duration_s", f"{el:.2f}"])
            w.writerow(["map_odom_updates", self.n_map_odom])
            w.writerow(["map_odom_hz", f"{self.n_map_odom / el if el else 0:.3f}"])
            w.writerow(["jumps", len(jumps)])
            w.writerow(["max_jump_m", f"{max(jumps) if jumps else 0:.3f}"])
            w.writerow(["robot_teleports", len(tele)])
            w.writerow(["max_teleport_m", f"{max(tele) if tele else 0:.3f}"])
            w.writerow(["max_gap_s", f"{self.max_gap:.3f}"])
            w.writerow(["robot_path_m", f"{self.robot_path:.2f}"])
            w.writerow([])
            w.writerow(["t_rel", "kind", "value"])
            for t, k, v in self.events:
                w.writerow([f"{t - self.t0:.3f}", k, f"{v:.3f}"])
        return path


def compare(files):
    rows = []
    for fn in files:
        d = {}
        with open(fn) as f:
            for r in csv.reader(f):
                if len(r) == 2:
                    d[r[0]] = r[1]
        rows.append((os.path.basename(fn), d))
    keys = ["duration_s", "map_odom_hz", "map_odom_updates", "jumps", "max_jump_m",
            "robot_teleports", "max_teleport_m", "max_gap_s", "robot_path_m"]
    w = max(len(k) for k in keys) + 2
    print("\n" + "=" * (w + 18 * len(rows)))
    print("重定位 A/B 对比".ljust(w) + "".join(f"{n[:16]:>18}" for n, _ in rows))
    print("-" * (w + 18 * len(rows)))
    for k in keys:
        print(k.ljust(w) + "".join(f"{d.get(k, '-')[:16]:>18}" for _, d in rows))
    print("=" * (w + 18 * len(rows)))
    print("怎么看：Hz 越高=修正越频繁；jumps/max_jump 越小越好；")
    print("        robot_teleports>0 = 机器人在地图里瞬移（用户会感觉定位跳了）；")
    print("        max_gap 大 = 有一段时间完全没有修正（跟丢或卡住）。")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="run")
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--out", default=os.path.expanduser("~"))
    ap.add_argument("--rate", type=float, default=5.0, help="打印汇总的频率")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--base-frame", default="base_footprint")
    ap.add_argument("--jump", type=float, default=0.3, help="判为跳变的阈值（米）")
    ap.add_argument("--compare", nargs="*", default=None,
                    help="对比模式：给若干个 CSV")
    args = ap.parse_args()

    if args.compare is not None:
        files = []
        for p in args.compare:
            files.extend(sorted(glob.glob(p)) or [p])
        if not files:
            print("没有找到 CSV")
            return 1
        compare(files)
        return 0

    if not _ROS_OK:
        print("采集模式需要 ROS 环境：先 source /opt/ros/humble/setup.bash "
              "（zsh 用 setup.zsh）")
        return 1
    rclpy.init()
    node = LocEval(args)
    print(f"开始采集 [{args.tag}]：等 {args.map_frame}->{args.odom_frame} "
          f"（{args.duration:.0f}s，Ctrl-C 可提前结束）", flush=True)
    t_end = time.time() + args.duration if args.duration > 0 else float("inf")
    t_next = time.time() + args.rate
    try:
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() >= t_next:
                t_next = time.time() + args.rate
                node.report()
                if node.n_map_odom == 0 and not node.warned_no_tf:
                    node.warned_no_tf = True
                    print("  （还没看到 map->odom：重定位节点没起来？初始位姿还没给？"
                          "initialpose 是在给位姿前就发了？）", flush=True)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，提前结束")
    node.report(final=True)
    path = os.path.join(
        args.out, f"jt128_loc_eval_{args.tag}_{datetime.now():%Y%m%d_%H%M%S}.csv")
    node.dump(path)
    print(f"已写出 {path}")
    print(f"对比: python3 ~/jt128/loc_eval.py --compare ~/{os.path.basename(path)} "
          f"~/jt128_loc_eval_<另一个方案>_*.csv")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
