#!/usr/bin/env bash
# 硬件无关回归测试：jt128_livox_compat（真机接 small_point_lio 的适配层）
#
# 链路：合成禾赛布局点云（x,y,z,intensity + ring + timestamp=**秒**）
#         -> jt128_livox_compat（补 tag/line，timestamp ×1e9 秒→纳秒）
#         -> 校验输出
#
# 校验点（写死的是 small_point_lio 的接收条件，见 livox_pointcloud2.h:25-37）：
#   1. point_step = 28（原 26 + tag + line）
#   2. tag & 0x3F == 0  → LIO 才会收这个点
#   3. line == ring & 0xFF
#   4. 逐点 timestamp 已是**纳秒**：跨度 ≈ 1e8（合成器一帧 0.1 s）
#
# 用法：./compat_selftest.sh          （退出码 0=通过，1=失败）
set -uo pipefail

JT128_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HESAI_WS="${HESAI_WS:-$HOME/hesai_ws}"
DDS_PROFILE="${JT128_DDS_PROFILE:-$JT128_DIR/sim/fastdds_large_msg.xml}"
HS_TOPIC="/jt128_selftest/hs_points"
LIVOX_TOPIC="/jt128_selftest/livox_points"

ok()   { printf '\033[32m   ✅ %s\033[0m\n' "$1"; }
bad()  { printf '\033[31m   ❌ %s\033[0m\n' "$1"; FAIL=1; }
info() { printf '   %s\n' "$1"; }
FAIL=0
PIDS=()

cleanup() { for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done; sleep 1; for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null; done; }
trap cleanup EXIT

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" 2>/dev/null || { bad "找不到 ROS setup"; exit 1; }
source "$HESAI_WS/install/setup.bash" 2>/dev/null || { bad "找不到 $HESAI_WS/install/setup.bash（先编译）"; exit 1; }
set -u
export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE"

printf '\033[1m== jt128_livox_compat 回归测试（不需要雷达）==\033[0m\n'

# 1) 合成禾赛布局点云
python3 "$JT128_DIR/sim/jt128_fake_hesai_cloud.py" --ros-args \
  -p topic:="$HS_TOPIC" -p points_per_frame:=20000 -p ts_base_sec:=1000.0 -p ts_span_sec:=0.1 \
  > /tmp/jt128_selftest_pub.log 2>&1 &
PIDS+=($!)
sleep 3
if pgrep -P "${PIDS[0]}" >/dev/null 2>&1 || kill -0 "${PIDS[0]}" 2>/dev/null; then
  ok "合成点云已启动（$HS_TOPIC，timestamp 1000.000~1000.100 秒）"
else
  bad "合成点云没起来，看 /tmp/jt128_selftest_pub.log"; exit 1
fi

# 2) 适配层（默认 timestamp_scale=1e9）
ros2 run jt128_livox_compat jt128_livox_compat_node --ros-args \
  -p in_topic:="$HS_TOPIC" -p out_topic:="$LIVOX_TOPIC" -p relay_imu:=false \
  > /tmp/jt128_selftest_compat.log 2>&1 &
PIDS+=($!)
sleep 3

# 3) 校验
timeout 40 python3 - "$LIVOX_TOPIC" <<'PY'
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

topic = sys.argv[1]
class Grab(Node):
    def __init__(self):
        super().__init__('jt128_compat_selftest')
        self.m = None
        self.create_subscription(PointCloud2, topic, self.cb,
            QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
    def cb(self, m):
        if self.m is None:
            self.m = m

rclpy.init(); n = Grab()
for _ in range(60):
    rclpy.spin_once(n, timeout_sec=0.5)
    if n.m: break
m = n.m
if m is None:
    print("   ❌ 20 秒内没收到输出点云"); n.destroy_node(); rclpy.shutdown(); sys.exit(1)

step, cnt, buf = m.point_step, m.width, bytes(m.data)
names = [f.name for f in m.fields]
def col(dtype, off):
    return np.ndarray(shape=(cnt,), dtype=np.dtype(dtype), buffer=buf, offset=off, strides=(step,))

fails = []
def chk(cond, msg):
    print(("   ✅ " if cond else "   ❌ ") + msg)
    if not cond: fails.append(msg)

chk(step == 28, f"point_step = {step}（期望 28 = 26 + tag + line）")
chk('tag' in names and 'line' in names, "输出含 tag / line 字段：" + ", ".join(names))
if 'tag' in names and 'line' in names:
    tag, line = col('u1', 26), col('u1', 27)
    ring = col('<u2', 16)
    chk(bool((tag & 0x3F == 0).all()), "tag & 0x3F == 0（small_point_lio 的接收条件）")
    chk(bool((line == (ring & 0xFF)).all()), "line == ring & 0xFF")
    ts = col('<f8', 18)
    span = float(ts.max() - ts.min())
    chk(abs(span - 1e8) < 1e6, f"逐点时间戳跨度 = {span:.0f}（期望 ≈1e8 纳秒；修复前是 0.1）")
    chk(9e11 < float(ts.min()) < 1.1e12,
        f"时间戳绝对量级 = {float(ts.min()):.0f}（期望 ≈1e12 = 1000 s × 1e9）")
n.destroy_node(); rclpy.shutdown()
sys.exit(1 if fails else 0)
PY
CHK=$?

echo
if [ "$CHK" = 0 ] && [ "$FAIL" = 0 ]; then
  printf '\033[32m== 通过：适配层输出符合 small_point_lio 的要求 ==\033[0m\n'
  exit 0
else
  printf '\033[31m== 失败（日志：/tmp/jt128_selftest_{pub,compat}.log）==\033[0m\n'
  exit 1
fi
