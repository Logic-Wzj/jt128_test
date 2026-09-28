#!/usr/bin/env python3
"""JT128 点云内容深检：字段布局 / 距离分布 / 通道覆盖 / 反射率 / 时间戳跨度。

    python3 pc_stats.py                       # 默认 /lidar_points，等 20 秒
    python3 pc_stats.py --topic /livox/lidar
    python3 pc_stats.py --json                # 只输出 JSON（给脚本用）

看什么：
  * 字段布局要对得上驱动版本（x,y,z,intensity,ring,timestamp...），point_step 26
  * 距离应有真实分布（室内地面/墙面几米，量程 0.5~60 m）；全零或全超大 = 解包不对
  * ring 应覆盖 128 条通道（机械式 128 线）
  * timestamp 是逐点纳秒，跨度应接近一帧时间（10 Hz → ~100 ms）
"""
import argparse
import json
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from sensor_msgs.msg import PointCloud2

DT = {1: 'int8', 2: 'uint8', 3: 'int16', 4: 'uint16',
      5: 'int32', 6: 'uint32', 7: 'float32', 8: 'float64'}


class Grab(Node):
    def __init__(self, topic, timeout):
        super().__init__('jt128_pc_stats')
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.msg = None
        self.create_subscription(PointCloud2, topic, self._cb, qos)
        self.deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)

    def _cb(self, m):
        if self.msg is None:
            self.msg = m

    def done(self):
        return self.msg is not None or self.get_clock().now().nanoseconds > self.deadline


def field_array(buf, n, point_step, f):
    return np.ndarray(shape=(n,), dtype=np.dtype(DT[f.datatype]), buffer=buf,
                      offset=f.offset, strides=(point_step,))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/lidar_points')
    ap.add_argument('--timeout', type=float, default=20.0)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    node = Grab(a.topic, a.timeout)
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.5)
    m = node.msg
    if m is None:
        print(f'❌ {a.timeout:.0f} 秒内没收到 {a.topic} 的点云')
        node.destroy_node(); rclpy.shutdown(); return 1

    n = m.width * m.height
    buf = bytes(m.data)
    names = {f.name: f for f in m.fields}
    out = {
        'topic': a.topic, 'frame_id': m.header.frame_id,
        'stamp_sec': m.header.stamp.sec, 'width': m.width, 'height': m.height,
        'point_step': m.point_step, 'row_step': m.row_step,
        'is_dense': bool(m.is_dense), 'data_bytes': len(buf),
        'fields': {f.name: {'offset': f.offset, 'type': DT.get(f.datatype, f.datatype),
                            'count': f.count} for f in m.fields},
    }

    if 'x' in names and 'y' in names and 'z' in names:
        x = field_array(buf, n, m.point_step, names['x']).astype(np.float64)
        y = field_array(buf, n, m.point_step, names['y']).astype(np.float64)
        z = field_array(buf, n, m.point_step, names['z']).astype(np.float64)
        d = np.sqrt(x * x + y * y + z * z)
        valid = d > 0.01
        out['points'] = n
        out['valid_points'] = int(valid.sum())
        out['zero_points'] = int((~valid).sum())
        if valid.any():
            dv = d[valid]
            out['dist_m'] = {'min': round(float(dv.min()), 2),
                             'p50': round(float(np.median(dv)), 2),
                             'p99': round(float(np.percentile(dv, 99)), 2),
                             'max': round(float(dv.max()), 2)}
            out['over_60m'] = int((dv > 60).sum())
            out['under_0.5m'] = int((dv < 0.5).sum())
    if 'ring' in names:
        r = field_array(buf, n, m.point_step, names['ring'])
        out['ring'] = {'min': int(r.min()), 'max': int(r.max()),
                       'unique': int(np.unique(r).size)}
    if 'intensity' in names:
        i = field_array(buf, n, m.point_step, names['intensity']).astype(np.float64)
        out['intensity'] = {'min': float(i.min()), 'max': float(i.max()),
                            'mean': round(float(i.mean()), 2)}
    if 'timestamp' in names:
        t = field_array(buf, n, m.point_step, names['timestamp']).astype(np.float64)
        span_ms = (t.max() - t.min()) / 1e6
        out['timestamp'] = {'min': float(t.min()), 'max': float(t.max()),
                            'span_ms': round(float(span_ms), 3),
                            'monotonic_ratio': round(float((np.diff(t) >= 0).mean()), 3)}

    node.destroy_node(); rclpy.shutdown()

    if a.json:
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return 0

    print(f"话题 {out['topic']}  frame_id={out['frame_id']}  "
          f"{out['width']}×{out['height']}  point_step={out['point_step']}  "
          f"data={out['data_bytes']/1e6:.2f} MB  is_dense={out['is_dense']}")
    print("字段：" + ", ".join(f"{k}@{v['offset']}:{v['type']}"
                              for k, v in out['fields'].items()))
    if 'dist_m' in out:
        print(f"点数 {out['points']}  有效 {out['valid_points']}  "
              f"全零点 {out['zero_points']}")
        dm = out['dist_m']
        print(f"距离(m) min={dm['min']} 中位={dm['p50']} p99={dm['p99']} max={dm['max']}  "
              f"| >60m: {out['over_60m']}  <0.5m: {out['under_0.5m']}")
    if 'ring' in out:
        print(f"通道 ring: {out['ring']['min']}~{out['ring']['max']} "
              f"（共 {out['ring']['unique']} 条）")
    if 'intensity' in out:
        it = out['intensity']
        print(f"反射率: min={it['min']:.0f} max={it['max']:.0f} 均值={it['mean']:.1f}")
    if 'timestamp' in out:
        ts = out['timestamp']
        print(f"逐点时间戳: 跨度 {ts['span_ms']:.2f} ms  递增比例 {ts['monotonic_ratio']:.2f}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
