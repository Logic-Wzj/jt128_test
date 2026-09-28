#!/usr/bin/env bash
# ============================================================================
# JT128 雷达一键启动/检查脚本（纯 bash，不依赖 zsh）
#
# 设计目标：**可移植**——放哪台机器（含机器人小电脑）都能跑，不写死任何
#           绝对路径、不写死网卡名、不依赖 zsh。所有路径都能用环境变量覆盖。
#
# 用法：
#   ./launch.sh                 # 状态总览（网卡/IP/雷达/驱动/仿真 一次看全）
#   ./launch.sh help            # 全部命令
#
#   ./launch.sh net [网卡] [主机IP]   # 配主机 IP（需要 sudo）
#   ./launch.sh check                 # 裸 UDP 收包统计（证明雷达在发数据）
#   ./launch.sh driver                # 只起禾赛驱动（今天测雷达用这个）
#   ./launch.sh verify                # 驱动点云逐项检查
#   ./launch.sh mem                   # 内存/CPU/FD/内核缓冲
#   ./launch.sh gui                   # 上位机 LidarUtilities
#   ./launch.sh sim | simstock | hil  # 仿真相关（今天用不到）
#   ./launch.sh fixpaths              # 修复驱动 config.yaml 里的绝对路径（换机必做）
#   ./launch.sh ps | down             # 看残留 / 清理
#
# 环境变量（都有合理默认，按需覆盖）：
#   JT128_IFACE     雷达网卡名（默认自动探测）
#   JT128_HOST_IP   主机 IP（默认 192.168.1.100）
#   JT128_RADAR_IP  雷达 IP（默认 192.168.1.201）
#   HESAI_WS        禾赛驱动工作区（默认 $HOME/hesai_ws）
#   HESAI_REPO      禾赛驱动源码（默认 $HOME/HesaiLidar_ROS_2.0）
#   COD_WS          cod 工作区（默认 $HOME/cod_-rm2026_-navigation）
#   ROS_DISTRO      ROS 发行版（默认自动探测 /opt/ros 下第一个）
#   JT128_APP_SRC   上位机 .out 的来源目录（可选）
# ============================================================================

set -u

# ---------- 定位自己（不写死路径）----------
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}" 2>/dev/null || echo "${BASH_SOURCE[0]}")"
JT128_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
export JT128_DIR

HESAI_WS="${HESAI_WS:-$HOME/hesai_ws}"
HESAI_REPO="${HESAI_REPO:-$HOME/HesaiLidar_ROS_2.0}"
COD_WS="${COD_WS:-$HOME/cod_-rm2026_-navigation}"
HOST_IP="${JT128_HOST_IP:-192.168.1.100}"
RADAR_IP="${JT128_RADAR_IP:-192.168.1.201}"
DDS_PROFILE="${JT128_DDS_PROFILE:-$JT128_DIR/sim/fastdds_large_msg.xml}"

# ROS 发行版：优先 $ROS_DISTRO，其次自动探测
detect_distro() {
  if [ -n "${ROS_DISTRO:-}" ]; then echo "$ROS_DISTRO"; return; fi
  local d
  for d in /opt/ros/*; do [ -d "$d" ] && { basename "$d"; return; }; done
}
DISTRO="$(detect_distro)"
[ -n "${DISTRO:-}" ] || DISTRO="humble"

# 网卡：优先 $JT128_IFACE，其次带 192.168.1.x 的网卡，其次默认路由网卡，最后第一块有线网卡
detect_iface() {
  local i
  if [ -n "${JT128_IFACE:-}" ]; then echo "$JT128_IFACE"; return; fi
  i="$(ip -o -4 addr show 2>/dev/null | awk '$4 ~ /^192\.168\.1\./ {print $2; exit}')"
  [ -n "$i" ] && { echo "$i"; return; }
  i="$(ip route show default 2>/dev/null | awk '{print $5; exit}')"
  [ -n "$i" ] && { echo "$i"; return; }
  for i in $(ls /sys/class/net 2>/dev/null); do
    case "$i" in lo|docker*|virbr*|veth*|br-*|wl*|Meta) continue ;; esac
    [ -e "/sys/class/net/$i/device" ] && { echo "$i"; return; }
  done
}

say()  { printf "\n\033[1m== %s ==\033[0m\n" "$1"; }
info() { printf "   %s\n" "$1"; }
err()  { printf "\033[31m   ✗ %s\033[0m\n" "$1"; }

# ROS 的 setup.*sh 会引用未绑定变量，source 前后要关/开 set -u
source_ros() {
  local f="/opt/ros/$DISTRO/setup.bash"
  [ -f "$f" ] || { err "找不到 $f（设 ROS_DISTRO 试试）"; return 1; }
  set +u; # shellcheck disable=SC1090
  source "$f"; set -u
}
source_ws() {  # $1=工作区目录
  local f="$1/install/setup.bash"
  [ -f "$f" ] || return 1
  set +u; # shellcheck disable=SC1090
  source "$f"; set -u
}

sim_running() { pgrep -f "ros2 launch cod_sim" >/dev/null 2>&1; }

# ---------- 修复驱动 config.yaml 的绝对路径 ----------
fixpaths() {
  local cfg="$HESAI_REPO/config/config.yaml"
  if [ ! -f "$cfg" ]; then err "找不到 $cfg（HESAI_REPO 设对了吗？）"; return 1; fi
  python3 - "$cfg" "$HESAI_REPO" <<'PY'
import os, re, sys
cfg, repo = sys.argv[1], sys.argv[2]
s = open(cfg, encoding="utf-8").read()
base = f"{repo}/src/driver/HesaiLidar_SDK_2.0/correction"
targets = {
    "correction_file_path": f"{base}/angle_correction/JT128_Angle Correction File.csv",
    "firetimes_path":        f"{base}/firetime_correction/JT128_Firetime Correction File.csv",
}
changed = []
for key, want in targets.items():
    m = re.search(r"(\n        " + key + r": )\"([^\"]*)\"", s)
    if not m:
        print(f"   ⚠️ 没找到 {key}"); continue
    cur = m.group(2)
    if cur == want:
        print(f"   ✓ {key} 已是本机路径"); continue
    if os.path.isfile(cur) and not os.path.isfile(want):
        print(f"   ✓ {key} 当前路径有效，不动"); continue
    if not os.path.isfile(want):
        print(f"   ⚠️ {key} 目标文件不存在：{want}（保持原样）"); continue
    s = s[:m.start()] + m.group(1) + f'"{want}"' + s[m.end():]
    changed.append(key)
open(cfg, "w", encoding="utf-8").write(s)
if changed:
    print("   ✅ 已改写：" + ", ".join(changed))
PY
  # 功能项：从 GitHub 克隆来的上游 config.yaml 不含这些改动，缺了会卡住或看不到点云
  python3 - "$cfg" <<'PY'
import re, sys
cfg = sys.argv[1]
s = open(cfg, encoding="utf-8").read()
edits, notes = [], []

def patch_val(pat, want, label):
    """只改配置值本身（保留行尾注释），已是目标值就跳过；幂等。"""
    global s
    m = re.search(pat, s, re.M)
    if not m:
        notes.append(f"{label}（没找到，跳过）"); return
    if m.group(2) == want:
        notes.append(f"{label} 已符合要求"); return
    s = s[:m.start(2)] + want + s[m.end(2):]
    edits.append(label)

# 点云/IMU 的 frame_id：RViz 固定坐标系、verify、兼容层都按这个找
patch_val(r"^(\s+ros_frame_id: )(\S+)", "front_jt128", "ros_frame_id→front_jt128")
# 上游默认 -1 = 初始化时无限阻塞等 PTC 连接（雷达 PTC 不通就永远起不来）
patch_val(r"^(\s+ptc_connect_timeout: )(\S+)", "3", "ptc_connect_timeout→3")
# 占位符路径指向不存在的文件会刷 LogError，置空
patch_val(r'^(\s+channel_fov_filter_path: )("[^"]*")', '""', "channel_fov_filter_path→空")
# 直连场景让 SDK 按源 IP 收包，不 join 组播组
patch_val(r'^(\s+multicast_ip_address: )("[^"]*"|\S+)', '""', "multicast_ip_address→空")

open(cfg, "w", encoding="utf-8").write(s)
if edits:
    print("   ✅ 已改写：" + "、".join(edits))
for n in notes:
    print(f"   · {n}")
PY
}

# ---------- 状态总览 ----------
status() {
  local iface; iface="$(detect_iface)"
  say "环境"
  info "脚本目录 : $JT128_DIR"
  info "ROS      : $DISTRO"
  info "驱动工作区: $HESAI_WS $([ -f "$HESAI_WS/install/setup.bash" ] && echo '✓' || echo '✗ 未编译')"
  info "驱动源码 : $HESAI_REPO $([ -d "$HESAI_REPO" ] && echo '✓' || echo '✗ 不存在')"
  info "cod 工作区: $COD_WS $([ -f "$COD_WS/install/setup.bash" ] && echo '✓' || echo '✗（只影响仿真/接管导航，今天用不到）')"

  say "网络（雷达网段）"
  if [ -n "${iface:-}" ]; then
    local addrs; addrs="$(ip -4 -o addr show dev "$iface" 2>/dev/null | awk '{print $4}' | tr '\n' ' ')"
    info "网卡     : $iface   地址: ${addrs:-（无 IPv4）}"
    local route; route="$(ip route get "$RADAR_IP" 2>&1 | head -1)"
    info "到雷达路由: $route"
    case "$route" in
      *"dev Meta"*) err "被 Clash TUN 劫持了！先关闭 TUN 或把 192.168.1.0/24 加进绕过列表" ;;
    esac
    case "$addrs" in
      *"$HOST_IP"*) : ;;
      *) info "提示：本机还没有 $HOST_IP → 跑  ./launch.sh net  （需要 sudo）" ;;
    esac
  else
    err "没探测到有线网卡（用 JT128_IFACE=xxx 指定）"
  fi

  say "进程"
  if pgrep -f "hesai_ros_driver" >/dev/null 2>&1; then info "禾赛驱动 : 在跑"; else info "禾赛驱动 : 没跑（./launch.sh driver 启动）"; fi
  if sim_running; then info "仿真     : 在跑（注意：同时只跑一个）"; else info "仿真     : 没跑"; fi
  local pub
  if source_ros >/dev/null 2>&1; then
    pub="$(timeout 6 ros2 topic info /clock --no-daemon 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')"
    info "/clock   : 发布者 ${pub:-0} 个 $([ "${pub:-0}" = "0" ] && echo '（0 = 没有仿真在跑）')"
  fi

  say "下一步"
  info "接线/上电后：./launch.sh net  →  ./launch.sh check  →  ./launch.sh driver"
  info "全部命令   ：./launch.sh help"
}

usage() {
  cat <<EOF
JT128 一键脚本（bash）

  今天测雷达（推荐顺序）：
    ./launch.sh radar                 ★一键★ 检查环境+探包+起驱动（接好雷达后跑这个）
    ./launch.sh net                   配主机 IP（sudo；可传 网卡 主机IP）
    ./launch.sh check                 裸 UDP 收包（证明雷达在发数据；≈9000 包/s 双回波）
    ./launch.sh driver                只起禾赛驱动
    ./launch.sh verify                点云逐项检查（频率/点数/frame_id/丢包/资源）
    ./launch.sh rviz                  开 RViz 看点云（已配好话题/坐标系/大点云 DDS profile）
    ./launch.sh mem                   内存/CPU/FD/内核缓冲（默认 600 秒）
    ./launch.sh gui                   上位机 LidarUtilities（改回波模式/FOV 等）
    ./launch.sh compat                真机接入：把 JT128 适配成 Mid-360（供 cod 导航栈使用）

  换机器/首次：
    ./launch.sh fixpaths              修复 config.yaml 里的绝对路径（换机必做）
    ./launch.sh status                环境总览（同直接运行本脚本）

  仿真相关（今天用不到）：
    ./launch.sh simstock              原版仿真（cod_sim 默认参数）
    ./launch.sh sim                   仿真（障碍源换成点云）
    ./launch.sh hil [source:=fake]    雷达→仿真 注入（需先有仿真在跑）

  维护：
    ./launch.sh ps                    看残留进程 + /clock 状态
    ./launch.sh down                  清理驱动/中继/仿真残留

  环境变量覆盖：JT128_IFACE / JT128_HOST_IP / HESAI_WS / HESAI_REPO / COD_WS / ROS_DISTRO
EOF
}

# ---------- 各子命令 ----------
cmd_net() {
  local iface="${1:-}"; [ -n "$iface" ] || iface="$(detect_iface)"
  local ip="${2:-$HOST_IP}"
  [ -n "${iface:-}" ] || { err "没探测到网卡，请指定：./launch.sh net <网卡> <主机IP>"; return 1; }
  info "网卡=$iface  主机IP=$ip"
  exec "$JT128_DIR/setup_host.sh" "$iface" "$ip"
}

cmd_check() {
  local iface; iface="$(detect_iface)"
  exec python3 "$JT128_DIR/jt128_check.py" ${iface:+--iface "$iface"} "$@"
}

cmd_driver() {
  fixpaths
  source_ros || return 1
  source_ws "$HESAI_WS" || { err "找不到 $HESAI_WS/install/setup.bash（先编译驱动）"; return 1; }
  if [ -f "$HESAI_REPO/launch/jt128_test.py" ]; then
    exec ros2 launch "$HESAI_REPO/launch/jt128_test.py" "$@"
  fi
  exec ros2 run hesai_ros_driver hesai_ros_driver_node "$@"
}

cmd_verify() {
  MODE="${MODE:-driver}" exec bash "$JT128_DIR/jt128_verify.sh" "$@"
}

cmd_compat() {
  # 真机接入：把 JT128 变成 Mid-360 的样子（/livox/lidar + /livox/imu + base_link->livox_frame）
  source_ros || return 1
  source_ws "$HESAI_WS" || { err "找不到 $HESAI_WS/install/setup.bash（先编译）"; return 1; }
  info "适配层：JT128 -> /livox/lidar + /livox/imu + base_link->livox_frame"
  info "起完再启动 cod 的 singlenav_launch.py（cpp_lidar_filter / small_point_lio / nav2 照旧）"
  exec ros2 launch jt128_livox_compat compat.launch.py "$@"
}

cmd_radar() {
  # 一键（雷达测试）：环境 → IP → 代理劫持 → 探包 → 起驱动
  local iface; iface="$(detect_iface)"
  say "一键启动：JT128 雷达测试"

  [ -n "${iface:-}" ] || { err "没探测到有线网卡（用 JT128_IFACE=xxx 指定）"; return 1; }
  local addrs; addrs="$(ip -4 -o addr show dev "$iface" 2>/dev/null | awk '{print $4}' | tr '\n' ' ')"
  info "网卡 $iface : ${addrs:-（无 IPv4）}"

  case "$addrs" in
    *192.168.1.*) : ;;
    *) err "本机还没有 192.168.1.x 地址 —— 先跑：sudo ./launch.sh net"
       info "（网卡名会自动探测；要指定：sudo ./launch.sh net <网卡> 192.168.1.100）"; return 1 ;;
  esac

  local route; route="$(ip route get "$RADAR_IP" 2>&1 | head -1)"
  case "$route" in
    *"dev Meta"*) err "雷达 IP 被代理劫持：$route"
                  info "关掉 TUN，或把 192.168.1.0/24 加进绕过列表"; return 1 ;;
  esac
  info "到雷达路由：$route"

  say "探包（2 秒，看雷达是否在发数据）"
  if ! python3 "$JT128_DIR/jt128_check.py" ${iface:+--iface "$iface"} --seconds 2; then
    err "2 秒内没收到点云包 —— 先查供电(9~32V/≥2.6A)、M8 线缆、网口收发交叉"
    info "物理排查：sudo tcpdump -i $iface -n -c5 udp port 2368"
    return 1
  fi

  say "探包通过，启动禾赛驱动"
  cmd_driver "$@"
}

cmd_rviz() {
  source_ros || return 1
  source_ws "$HESAI_WS" >/dev/null 2>&1 || true
  local cfg="$JT128_DIR/sim/jt128.rviz"
  [ -f "$cfg" ] || { err "缺 $cfg"; return 1; }
  # 1.84MB/帧的大点云必须带这个 profile，否则 RViz 只收到 1~2 帧/s（本机实测）
  export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE"
  info "话题：/lidar_points    Fixed Frame：front_jt128（点云自带坐标系，不需要额外 TF）"
  info "DDS profile：$(basename "$DDS_PROFILE")（大点云必需）"
  info "看不到点云时依次查：驱动在跑吗 → 话题对不对 → Fixed Frame 是否设成 front_jt128"
  exec rviz2 -d "$cfg" "$@"
}

cmd_mem() {
  exec python3 "$JT128_DIR/memwatch.py" --name hesai_ros_driver "$@"
}

cmd_gui() {
  local app
  app="$(ls "$JT128_DIR"/app/*.out 2>/dev/null | head -1)"
  if [ -z "$app" ] && [ -n "${JT128_APP_SRC:-}" ]; then
    local src; src="$(ls "$JT128_APP_SRC"/*.out 2>/dev/null | head -1)"
    if [ -n "$src" ]; then
      mkdir -p "$JT128_DIR/app"; cp "$src" "$JT128_DIR/app/"; app="$(ls "$JT128_DIR/app"/*.out | head -1)"
      info "已从 $JT128_APP_SRC 拷到 app/"
    fi
  fi
  if [ -z "$app" ]; then
    err "没找到上位机 LidarUtilities（放在 $JT128_DIR/app/ 下，或设 JT128_APP_SRC 指到它的目录）"
    info "它只支持 x86-64；ARM 小电脑上跑不了，需在 x86 机器上改参数"
    return 1
  fi
  chmod +x "$app" 2>/dev/null
  exec "$app"
}

cmd_sim() {  # 障碍源=点云
  if sim_running; then err "已有仿真在跑（同时跑两个 Gazebo 会互相挤死）"; return 1; fi
  source_ros || return 1
  source_ws "$COD_WS" || { err "找不到 cod 工作区 install/setup.bash"; return 1; }
  local pf="$JT128_DIR/sim/gt_sim_jt128_params.yaml"
  [ -f "$pf" ] || { err "缺 $pf"; return 1; }
  info "点云参数文件：$pf"
  FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE" \
    exec ros2 launch cod_sim fzsd_sim_launch.py params_file:="$pf" "$@"
}

cmd_simstock() {  # 原版仿真
  if sim_running; then err "已有仿真在跑"; return 1; fi
  source_ros || return 1
  source_ws "$COD_WS" || return 1
  exec ros2 launch cod_sim fzsd_sim_launch.py "$@"
}

cmd_hil() {
  if ! sim_running; then err "没检测到仿真在跑 —— HIL 需要仿真的 /clock 与 TF"; return 1; fi
  source_ros || return 1
  source_ws "$HESAI_WS" || { err "找不到 $HESAI_WS/install/setup.bash"; return 1; }
  source_ws "$COD_WS" >/dev/null 2>&1 || true
  FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE" \
    exec ros2 launch "$JT128_DIR/launch/jt128_sim_hil.launch.py" "$@"
}

cmd_ps() {
  say "JT128 / 仿真 相关进程"
  ps -eo pid,pgid,etime,cmd 2>/dev/null \
    | grep -E "ros2 launch (cod_sim|$JT128_DIR)|ign gazebo|jt128_sim_relay|jt128_fake_cloud|hesai_ros_driver|lidar_filter_node|rviz2" \
    | grep -v grep \
    | awk '{printf "   pid=%-7s pgid=%-7s 运行=%-9s %s\n", $1, $2, $3, substr($0, index($0,$4), 95)}' \
    || info "（无）"
  local pub
  if source_ros >/dev/null 2>&1; then
    pub="$(timeout 6 ros2 topic info /clock --no-daemon 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')"
    info "/clock 发布者：${pub:-0}（0 = 没有仿真，或 Gazebo 已死）"
  fi
}

cmd_down() {
  info "清理中…"
  pkill -f "ros2 launch .*jt128"                2>/dev/null
  pkill -f "hesai_ros_driver[_]node"            2>/dev/null
  pkill -f "jt128_sim_rela[y]"                  2>/dev/null
  pkill -f "jt128_fake_clou[d]"                 2>/dev/null
  pkill -f "cpp_lidar_filter/lib"               2>/dev/null
  sleep 1
  local n; n="$(pgrep -cf 'hesai_ros_driver[_]node|jt128_sim_rela[y]' 2>/dev/null)"; n="${n:-0}"
  info "剩余 JT128 进程：$n"
  pkill -f "ig[n] gazebo" 2>/dev/null
  info "（仿真/仿真节点本脚本不动，需要就用 ./launch.sh ps 看 pgid 后自己决定）"
}

# ---------- 分发 ----------
case "${1:-status}" in
  status|"")      status ;;
  help|-h|--help) usage ;;
  net)            shift; cmd_net "$@" ;;
  check)          shift; cmd_check "$@" ;;
  driver)         shift; cmd_driver "$@" ;;
  radar|start|oneclick) shift; cmd_radar "$@" ;;
  verify)         shift; cmd_verify "$@" ;;
  mem)            shift; cmd_mem "$@" ;;
  gui)            cmd_gui ;;
  rviz)           shift; cmd_rviz "$@" ;;
  compat)         shift; cmd_compat "$@" ;;
  sim)            shift; cmd_sim "$@" ;;
  simstock)       shift; cmd_simstock "$@" ;;
  hil)            shift; cmd_hil "$@" ;;
  fixpaths)       say "修复驱动 config.yaml 绝对路径"; fixpaths ;;
  ps)             cmd_ps ;;
  down)           cmd_down ;;
  *)              err "未知命令：$1"; usage; exit 1 ;;
esac
