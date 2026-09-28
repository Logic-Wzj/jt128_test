#!/usr/bin/env bash
# JT128 一键验证：驱动/中继跑起来之后，逐项检查是否正常
#
# 两种模式：
#   真机（默认）：驱动 + cpp_lidar_filter
#       bash ~/jt128/jt128_verify.sh
#   仿真 HIL   ：驱动 + C++ 中继 -> 仿真 costmap
#       MODE=sim bash ~/jt128/jt128_verify.sh
#   仿真 HIL 干跑（没接雷达，source:=fake）
#       MODE=sim PROCS_OPTIONAL= bash ~/jt128/jt128_verify.sh
#
# 单项可用环境变量覆盖：LIDAR_TOPIC / FILTERED_TOPIC / FRAME / LOSS_TOPIC

set -u

MODE="${MODE:-real}"
WS_HESAI="${WS_HESAI:-$HOME/hesai_ws}"
WS_COD="${WS_COD:-$HOME/cod_-rm2026_-navigation}"

if [ "$MODE" = "driver" ]; then
  # 「只接雷达测雷达」模式：只查驱动的点云，不查滤波节点与 TF
  LIDAR_TOPIC="${LIDAR_TOPIC:-/lidar_points}"
  FILTERED_TOPIC=""
  FRAME="${FRAME:-front_jt128}"
  PROCS_REQUIRED="${PROCS_REQUIRED:-hesai_ros_driver_node}"
  PROCS_OPTIONAL="${PROCS_OPTIONAL-}"
  TITLE="纯雷达（只起禾赛驱动，不接机器人/仿真）"
elif [ "$MODE" = "sim" ]; then
  LIDAR_TOPIC="${LIDAR_TOPIC:-/lidar_points}"
  FILTERED_TOPIC="${FILTERED_TOPIC:-/jt128/points}"
  FRAME="${FRAME:-front_mid360}"
  PROCS_REQUIRED="${PROCS_REQUIRED:-jt128_sim_relay}"
  PROCS_OPTIONAL="${PROCS_OPTIONAL-hesai_ros_driver_node}"
  TITLE="仿真 HIL（真实雷达 -> 仿真 costmap）"
else
  LIDAR_TOPIC="${LIDAR_TOPIC:-/lidar_points}"
  FILTERED_TOPIC="${FILTERED_TOPIC:-/livox/lidar_filtered}"
  FRAME="${FRAME:-front_jt128}"
  PROCS_REQUIRED="${PROCS_REQUIRED:-hesai_ros_driver_node lidar_filter_node}"
  PROCS_OPTIONAL="${PROCS_OPTIONAL-}"
  TITLE="真机（驱动 + cpp_lidar_filter）"
fi
LOSS_TOPIC="${LOSS_TOPIC:-/lidar_packets_loss}"
BASE_FRAME="${BASE_FRAME:-base_link}"

# ROS 的 setup.bash 会引用未绑定变量（AMENT_TRACE_SETUP_FILES），先关 set -u
set +u
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
[ -f "$ROS_SETUP" ] || { echo "找不到 $ROS_SETUP（用 ROS_DISTRO 指定发行版）"; exit 1; }
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$WS_HESAI/install/setup.bash" 2>/dev/null || { echo "找不到 $WS_HESAI/install/setup.bash（驱动还没编译？）"; exit 1; }
# shellcheck disable=SC1091
source "$WS_COD/install/setup.bash" 2>/dev/null || echo "警告：没找到 cod 工作区 install/setup.bash"
set -u

OK=0; FAIL=0; WARN=0
ok()   { printf "  \033[32m✅ %s\033[0m\n" "$1"; OK=$((OK+1)); }
bad()  { printf "  \033[31m❌ %s\033[0m\n" "$1"; FAIL=$((FAIL+1)); }
warn() { printf "  \033[33m⚠️  %s\033[0m\n" "$1"; WARN=$((WARN+1)); }
info() { printf "     %s\n" "$1"; }

# 仿真是否在跑（有 /clock 才算）
sim_up=0
ros2 topic list 2>/dev/null | grep -qx "/clock" && sim_up=1

echo "==================== JT128 验证：$TITLE ===================="
echo "     模式 MODE=$MODE | 点云 $LIDAR_TOPIC -> $FILTERED_TOPIC | 期望 frame=$FRAME"
if [ "$MODE" = "sim" ]; then
  [ "$sim_up" = 1 ] && info "检测到仿真 /clock ✅" || info "未检测到仿真 /clock（仿真没起时 TF/丢包检查会降级为警告）"
fi

# ---------- 1. 进程 ----------
echo "[1] 进程"
for p in $PROCS_REQUIRED; do
  if pgrep -f "$p" >/dev/null; then
    pid=$(pgrep -f "$p" | head -1)
    rss=$(awk '/VmRSS/{print $2}' "/proc/$pid/status" 2>/dev/null)
    ok "$p 在跑 (pid=$pid, RSS=$(( ${rss:-0} / 1024 )) MB)"
  else
    bad "$p 没在跑"
  fi
done
for p in $PROCS_OPTIONAL; do
  pgrep -f "$p" >/dev/null && ok "$p 在跑" || warn "$p 没在跑（干跑/只测中继时正常）"
done

# ---------- 2. 话题 ----------
echo "[2] 话题"
for t in "$LIDAR_TOPIC" $FILTERED_TOPIC; do
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then
    pub=$(timeout 5 ros2 topic info "$t" 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')
    ok "$t 存在 (publisher=$pub)"
  else
    bad "$t 不存在"
  fi
done
if ros2 topic list 2>/dev/null | grep -qx "$LOSS_TOPIC"; then
  ok "$LOSS_TOPIC 存在（驱动在跑）"
else
  warn "$LOSS_TOPIC 不存在（驱动没跑时正常）"
fi

# ---------- 3. 频率 ----------
# 注意：ros2 topic hz 会打印多行 "average rate:"，必须只取最后一行
hz_of() {
  timeout 9 ros2 topic hz "$1" 2>/dev/null \
    | awk -F': ' '/average rate/{r=$2} END{gsub(/[^0-9.]/,"",r); print r}'
}
echo "[3] 发布频率"
for t in "$LIDAR_TOPIC" $FILTERED_TOPIC; do
  r=$(hz_of "$t")
  if [ -z "${r:-}" ]; then
    bad "$t 没有数据（话题空）"
  elif awk -v r="$r" 'BEGIN{exit !(r>8 && r<12)}'; then
    ok "$t ≈ ${r} Hz（10 Hz 工况）"
  elif awk -v r="$r" 'BEGIN{exit !(r>17 && r<23)}'; then
    ok "$t ≈ ${r} Hz（20 Hz 工况）"
  else
    bad "$t 频率异常：${r} Hz"
  fi
done

# ---------- 4. 点云内容 ----------
echo "[4] 点云内容"
w=$(timeout 8 ros2 topic echo --once --field width "$LIDAR_TOPIC" 2>/dev/null | head -1 | tr -dc '0-9')
h=$(timeout 8 ros2 topic echo --once --field height "$LIDAR_TOPIC" 2>/dev/null | head -1 | tr -dc '0-9')
fid=$(timeout 8 ros2 topic echo --once --field header.frame_id "$LIDAR_TOPIC" 2>/dev/null | grep -v WARNING | head -1 | tr -d '"\r')
if [ -n "${w:-}" ] && [ -n "${h:-}" ]; then
  pts=$((w*h))
  info "每帧点数 = $w × $h = $pts"
  case "$pts" in
    115200) ok "点数符合单回波 10 Hz（115200）" ;;
    230400) ok "点数符合双回波 10 Hz（230400）" ;;
    *)      bad "点数异常（既不是 115200 也不是 230400）→ 检查驱动回波模式是否与雷达一致" ;;
  esac
else
  bad "读不到点云 width/height"
fi
if [ -n "${fid:-}" ]; then
  [ "$fid" = "$FRAME" ] && ok "frame_id = $fid" || bad "frame_id = $fid（期望 $FRAME）"
else
  bad "读不到 frame_id"
fi

# ---------- 5. 丢包 ----------
echo "[5] 丢包统计"
if [ "$MODE" = "sim" ] && [ "$sim_up" = 0 ]; then
  warn "仿真没起，跳过"
else
  tot=$(timeout 8 ros2 topic echo --once --field total_packet_count "$LOSS_TOPIC" 2>/dev/null | head -1 | tr -dc '0-9')
  los=$(timeout 8 ros2 topic echo --once --field total_packet_loss_count "$LOSS_TOPIC" 2>/dev/null | head -1 | tr -dc '0-9')
  if [ -n "${tot:-}" ]; then
    los=${los:-0}
    info "累计收包 $tot，丢包 $los"
    [ "$los" = "0" ] && ok "丢包 0" || bad "有丢包：$los / $tot"
  else
    warn "读不到丢包统计"
  fi
fi

# ---------- 6. TF ----------
echo "[6] TF $BASE_FRAME -> $FRAME"
tfout=$(timeout 5 ros2 run tf2_ros tf2_echo "$BASE_FRAME" "$FRAME" 2>&1 | head -6)
if echo "$tfout" | grep -q "Translation"; then
  ok "TF 链通"
  echo "$tfout" | sed -n '2,5p' | sed 's/^/     /'
elif [ "$MODE" = "driver" ]; then
  warn "TF 未检查（纯雷达测试不需要坐标系链）"
elif [ "$MODE" = "sim" ] && [ "$sim_up" = 0 ]; then
  warn "TF 不通 —— 仿真没起，front_mid360 还没发布（先跑 jt128sim）"
else
  bad "TF 链不通（rviz/nav2 会报 frame 不存在）"
fi

# ---------- 7. 资源 ----------
echo "[7] 资源占用"
echo "     详细曲线：python3 ~/jt128/memwatch.py --name hesai_ros_driver --seconds 1800"
sampled=0
for p in hesai_ros_driver_node jt128_sim_relay lidar_filter_node; do
  pid=$(pgrep -f "$p" 2>/dev/null | head -1)
  [ -n "${pid:-}" ] || continue
  r0=$(awk '/VmRSS/{print $2}' "/proc/$pid/status" 2>/dev/null)
  info "$p RSS = $((${r0:-0} / 1024)) MB (pid=$pid)"
  sampled=1
done
if [ "$sampled" = 1 ]; then
  dpid=$(pgrep -f hesai_ros_driver_node 2>/dev/null | head -1)
  if [ -n "${dpid:-}" ]; then
    c0=$(awk '{print $14+$15}' "/proc/$dpid/stat" 2>/dev/null)
    r0=$(awk '/VmRSS/{print $2}' "/proc/$dpid/status")
    sleep 10
    if [ -d "/proc/$dpid" ]; then
      c1=$(awk '{print $14+$15}' "/proc/$dpid/stat")
      r1=$(awk '/VmRSS/{print $2}' "/proc/$dpid/status")
      info "驱动 10 秒：RSS $((r0/1024)) -> $((r1/1024)) MB | CPU ≈ $(( (c1-c0)/10 ))%（单核）"
      ok "资源采样完成"
    else
      bad "驱动在采样期间退出了"
    fi
  else
    ok "资源采样完成（只有中继在跑，没测驱动）"
  fi
else
  bad "没有任何进程可采样"
fi

echo "=========================================================="
printf "结果：\033[32m%d 通过\033[0m，\033[33m%d 警告\033[0m，\033[31m%d 失败\033[0m\n" "$OK" "$WARN" "$FAIL"
if [ "$FAIL" -eq 0 ]; then
  echo "→ 链路正常"
else
  echo "→ 点云没数据时按顺序查：Clash 是否放行 192.168.1.0/24 → 主机 IP → tcpdump → 驱动回波模式"
  echo "→ 仿真侧还要确认 FASTRTPS_DEFAULT_PROFILES_FILE 是否设置（不设大点云会被 DDS 丢掉）"
fi
exit "$FAIL"
