# JT128 测试环境（zsh 版）
# 用法：在 ~/.zshrc 里加一行  source ~/jt128/jt128.zsh  （或手动 source 本文件）
#
# 提供的命令：
#   jt128env      加载 ROS + 禾赛工作区 + cod 工作区
#   jt128up       起「驱动 + 滤波 + 静态TF」集成测试（方案 A，不动 cod 仓库）
#   jt128raw      只起驱动（不启滤波，测裸点云/占用）
#   jt128verify   跑一键验证（进程/话题/频率/点数/frame_id/丢包/TF/资源）
#   jt128mem      盯驱动内存/CPU/FD/内核缓冲
#   jt128check    裸 UDP 收包统计（不依赖 ROS，验证雷达是否在发数据）
#   jt128down     杀掉所有 JT128 相关进程（清理残留孤儿进程）
#   jt128status   看一眼当前跑着哪些 JT128 进程

# 自身所在目录：被 source 时用 zsh 的 %x 取当前文件路径。
# 以"本文件实际所在的目录"为准（目录改名/换位置、或环境里残留了旧的 JT128_DIR 都不会错）；
# 只有拿不到文件路径时才退回 $HOME/jt128。
_jt128_self="${(%):-%x}"
if [ -n "$_jt128_self" ] && [ -f "$_jt128_self" ]; then
  _jt128_dir="${_jt128_self:A:h}"
  if [ -n "${JT128_DIR:-}" ] && [ "${JT128_DIR}" != "${_jt128_dir}" ]; then
    echo "[jt128] JT128_DIR: ${JT128_DIR} -> ${_jt128_dir}（按本文件实际位置修正）"
  fi
  export JT128_DIR="${_jt128_dir}"
else
  export JT128_DIR="${JT128_DIR:-$HOME/jt128}"
fi
unset _jt128_self _jt128_dir
# 文档目录：2026-09-28 起所有 .md 从工具包里搬到同级目录（脚本不依赖文档，搬不搬都能跑）
export JT128_DOC_DIR="${JT128_DOC_DIR:-$HOME/jt128_文档}"
export HESAI_WS="${HESAI_WS:-$HOME/hesai_ws}"
export COD_WS="${COD_WS:-$HOME/cod_-rm2026_-navigation}"
export HESAI_SRC="${HESAI_SRC:-$HOME/HesaiLidar_ROS_2.0}"
# 防呆：一个系统上同时跑两个 Gazebo 会抢 gz-transport 端口，把先起的那个挤死
# （表现：Gazebo 进程消失、/clock 没有 publisher、仿真冻结但 ROS 节点还在）
# 这两个函数用来查/防重复启动 —— 本人踩过：起了第二个仿真，把用户正在跑的仿真弄死了。
function _jt128_sim_launch_pids() {
  pgrep -f "ros2 launch cod_sim" 2>/dev/null
}

function jt128ps() {
  echo "=== 仿真 / HIL 相关进程（pgid 相同的是同一棵树）==="
  ps -eo pid,pgid,etime,cmd --sort=start_time \
    | grep -E "ros2 launch cod_sim|ign gazebo|jt128_sim_relay|jt128_fake_cloud|hesai_ros_driver|static_transform_publisher|rviz2" \
    | grep -v grep \
    | awk '{printf "  pid=%-7s pgid=%-7s 运行=%-9s %s\n", $1, $2, $3, substr($0, index($0,$4), 95)}'
  echo "=== /clock 有没有发布者（没有 = Gazebo 已死，仿真冻结）==="
  timeout 6 ros2 topic info /clock --no-daemon 2>/dev/null | grep -i publisher || echo "  (查不到，ROS 环境可能没 source)"
}

# Fast DDS 大消息 profile：不在这里全局 export！
# 它会让**每个** ROS participant 都开一个 64 MB 共享内存段（跑整套导航就是几十个段），
# 所以只在实际需要发大点云的两条命令里按需注入（见 jt128hil / jt128sim）。
# 别的工具偶尔要收大点云时，用 jt128dds 在当前 shell 里显式打开。
export JT128_DDS_PROFILE="${JT128_DDS_PROFILE:-$JT128_DIR/sim/fastdds_large_msg.xml}"

function jt128dds() {
  # 在当前 shell 打开/关闭大消息 profile（rviz、ros2 topic echo 收 1.84MB 点云时用得上）
  if [ "${1:-on}" = "on" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$JT128_DDS_PROFILE"
    echo "[jt128] 已启用 DDS 大消息 profile：$FASTRTPS_DEFAULT_PROFILES_FILE"
    echo "        注意：本 shell 之后启动的每个 ROS 进程都会开 64 MB 共享内存段"
  else
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    echo "[jt128] 已关闭 DDS 大消息 profile"
  fi
}

function jt128env() {
  local distro="${ROS_DISTRO:-humble}"
  local ros_setup="/opt/ros/$distro/setup.zsh"
  if [ ! -f "$ros_setup" ]; then
    echo "[jt128] 找不到 $ros_setup —— 机器人上换了发行版就设 ROS_DISTRO，例如 export ROS_DISTRO=humble"
    return 1
  fi
  source "$ros_setup"
  source "$HESAI_WS/install/setup.zsh" || { echo "[jt128] 找不到 $HESAI_WS/install/setup.zsh"; return 1; }
  source "$COD_WS/install/setup.zsh" 2>/dev/null || echo "[jt128] 警告：cod 工作区没 source 上，滤波节点会起不来"
  echo "[jt128] 环境就绪：$distro + hesai_ws + cod"
}

# ---------- HIL：真实雷达接入仿真 ----------
function jt128sim() {
  # 仿真侧：障碍源换成点云（用参数文件，不改 cod 仓库）
  local busy=$(_jt128_sim_launch_pids)
  if [ -n "$busy" ]; then
    echo "[jt128] ⛔ 已经有仿真在跑了（PID: ${busy//$'\n'/ }），拒绝再起一个。用 jt128ps 查看。"
    return 1
  fi
  # 注意：仿真所需的 rmu_gazebo_simulator / pb2025_nav_bringup / fzsd2025_robot_description
  # 都由 cod 仓库里的 src/fzsd_vendor/ 提供并编译进 cod 工作区，**不需要**外部的 ~/fzsd2025
  # （fzsd_sim_launch.py 开头那句 "前置：source ~/fzsd2025/..." 的注释是过时的）
  jt128env >/dev/null || return 1
  echo "[jt128] 仿真侧 DDS profile: $JT128_DDS_PROFILE"
  FASTRTPS_DEFAULT_PROFILES_FILE="$JT128_DDS_PROFILE" \
    ros2 launch cod_sim fzsd_sim_launch.py \
      params_file:="$JT128_DIR/sim/gt_sim_jt128_params.yaml" "$@"
}

function jt128simstock() {
  # 原版仿真（不接 HIL）：用 cod_sim 自带参数，障碍源还是 rplidar 的 /scan
  # 用它先确认仿真本身是好的（local_costmap 能更新、controller_server 不崩）
  local busy=$(_jt128_sim_launch_pids)
  if [ -n "$busy" ]; then
    echo "[jt128] ⛔ 已经有仿真在跑了（PID: ${busy//$'\n'/ }），拒绝再起一个。"
    echo "        两个 Gazebo 会抢端口互相挤死。先关掉原来的，或直接用它。用 jt128ps 查看。"
    return 1
  fi
  jt128env >/dev/null || return 1
  echo "[jt128] 原版仿真（cod_sim 默认参数，无 HIL 注入）"
  ros2 launch cod_sim fzsd_sim_launch.py "$@"
}

function jt128hil() {
  # 雷达侧：驱动 + C++ 中继 -> /jt128/points（source:=fake 可无雷达干跑）
  if [ -z "$(_jt128_sim_launch_pids)" ]; then
    echo "[jt128] ⚠️  没检测到正在跑的仿真 —— HIL 依赖仿真的 /clock，点云会被 nav2 丢弃。"
    echo "        先另开终端跑 jt128sim（或 jt128simstock）。"
  fi
  jt128env >/dev/null || return 1
  FASTRTPS_DEFAULT_PROFILES_FILE="$JT128_DDS_PROFILE" \
    ros2 launch "$JT128_DIR/launch/jt128_sim_hil.launch.py" "$@"
}


function jt128up() {
  jt128env >/dev/null || return 1
  ros2 launch "$JT128_DIR/launch/jt128_nav_test.py" "$@"
}

function jt128raw() {
  jt128env >/dev/null || return 1
  ros2 launch "$HESAI_SRC/launch/jt128_test.py" "$@"
}

function jt128verify() {
  bash "$JT128_DIR/jt128_verify.sh" "$@"
}

function jt128mem() {
  python3 "$JT128_DIR/memwatch.py" --name hesai_ros_driver "$@"
}

function jt128route() {
  # 检查雷达 IP 有没有被 Clash(TUN/fake-ip) 劫持
  # 网卡名不写死：优先用 $JT128_IFACE，其次自动找带 192.168.1.x 地址的网卡
  local iface="${JT128_IFACE:-}"
  if [ -z "$iface" ]; then
    iface=$(ip -o -4 addr show 2>/dev/null | awk '$4 ~ /^192\.168\.1\./ {print $2; exit}')
  fi
  iface="${iface:-enp131s0}"
  local out
  out=$(ip route get 192.168.1.201 2>&1 | head -1)
  echo "ip route get 192.168.1.201:"
  echo "  $out"
  if echo "$out" | grep -q "dev Meta"; then
    echo "  ❌ 被 Clash 劫持了 —— 打开 TUN 模式时才会这样"
    echo "     放行要写在「全局扩展配置(Merge)」的 tun.route-exclude-address 里才持久；"
    echo "     改 clash-verge.yaml 会被 Verge 重新生成覆盖。"
    echo "     参考：$JT128_DOC_DIR/接上雷达后.md 第①节"
    return 1
  elif echo "$out" | grep -q "dev $iface"; then
    echo "  ✅ 没被劫持（走本地网卡）"
  else
    echo "  ⚠️  没被 Meta 劫持，但也没走 $iface —— 可能还没配 192.168.1.x/24"
    echo "     配：sudo ~/jt128/setup_host.sh $iface 192.168.1.100"
  fi
  # 顺带看一眼有没有 TUN 残留（TUN 关了但规则还在 = 黑洞）
  if ip rule list 2>/dev/null | grep -q "lookup 2022"; then
    echo "  ⚠️  策略路由里还有 table 2022 的残留规则（TUN 关闭后没清干净会造成丢包）："
    ip rule list | grep 2022 | sed 's/^/     /'
    echo "     清不掉就重启 clash-verge 服务或重启系统。"
  fi
  return 0
}

function jt128check() {
  python3 "$JT128_DIR/jt128_check.py" "$@"
}

function jt128status() {
  # pgrep -c 无匹配时也会输出 0，所以不要用 `|| echo 0`（会变成两行）
  local n
  n=$(pgrep -fc "hesai_ros_driver[_]node" 2>/dev/null); n=${n:-0}
  echo "驱动进程: $n 个"
  ps -eo pid,etime,rss,cmd 2>/dev/null | grep -E "hesai_ros_driver[_]node|lidar[_]filter_node|static_transform_publisher" | grep -v grep \
    | awk '{printf "  pid=%s 运行=%s RSS=%dMB  %s\n", $1, $2, $3/1024, $6}'
}

# 注意两点：
#  1) 不要用 `timeout ros2 launch ...`：timeout 只杀 launch 父进程，节点会被 init 收养变孤儿一直跑。
#  2) pkill 的 pattern 用 [x] 写法，否则 pattern 文本会出现在本进程 argv 里导致 pkill 杀掉自己。
function jt128down() {
  local before after
  before=$(pgrep -fc "hesai_ros_driver[_]node|lidar[_]filter_node|ros2[ ]launch.*jt128" 2>/dev/null); before=${before:-0}
  pkill -f "ros2[ ]launch.*jt128" 2>/dev/null
  pkill -f "hesai_ros_driver[_]node" 2>/dev/null
  pkill -f "cpp_lidar_filter/lib/cpp_lidar_filter/lidar[_]filter_node" 2>/dev/null
  pkill -f "base_link_to[_]front_jt128" 2>/dev/null
  pkill -f "rviz2 -d .*cod_nav.rviz" 2>/dev/null
  sleep 1
  after=$(pgrep -fc "hesai_ros_driver[_]node|lidar[_]filter_node" 2>/dev/null); after=${after:-0}
  echo "[jt128] 清理完成：清理前 $before 个相关进程，现在还剩 $after 个"
  [ "$after" -gt 0 ] && echo "  还剩的话：pkill -9 -f hesai_ros_driver_node"
  return 0
}
