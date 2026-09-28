#!/usr/bin/env bash
# JT128 机器人主机环境依赖自检
#
# 用法：bash deps_check.sh
#       ROS_DISTRO=humble bash deps_check.sh   # 目标机是别的发行版时指定

set -u

DISTRO="${ROS_DISTRO:-}"
if [ -z "$DISTRO" ]; then
  for d in /opt/ros/*; do [ -d "$d" ] && DISTRO=$(basename "$d") && break; done
fi
DISTRO="${DISTRO:-humble}"
ROS_PREFIX="/opt/ros/$DISTRO"

OK=0; FAIL=0; WARN=0
ok()   { printf "  \033[32m✅ %s\033[0m\n" "$1"; OK=$((OK+1)); }
bad()  { printf "  \033[31m❌ %s\033[0m\n" "$1"; FAIL=$((FAIL+1)); }
warn() { printf "  \033[33m⚠️  %s\033[0m\n" "$1"; WARN=$((WARN+1)); }
hint() { printf "     → %s\n" "$1"; }
info() { printf "     %s\n" "$1"; }
have_lib() {   # 直接查文件（ldconfig -p 在部分机器上缓存不全）
  local n="$1" d
  for d in /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu /lib/aarch64-linux-gnu /usr/lib /usr/lib64; do
    [ -e "$d/$n" ] && return 0
  done
  return 1
}

echo "==================== JT128 环境自检 ===================="
echo "架构 $(uname -m) | 系统 $(lsb_release -ds 2>/dev/null || cat /etc/os-release | grep PRETTY | cut -d= -f2) | Python $(python3 -V 2>&1 | cut -d' ' -f2)"

echo
echo "[1] ROS 2"
if [ -d "$ROS_PREFIX" ]; then
  ok "ROS 2 $DISTRO 在 $ROS_PREFIX"
else
  bad "找不到 $ROS_PREFIX"
  hint "设 ROS_DISTRO=<目标发行版> 再跑，或先装 ROS 2"
fi

# 驱动编译/运行需要的 ROS 包
PKGS="ament_cmake ament_index_cpp rclcpp rclcpp_action rcl_interfaces rcutils \
      sensor_msgs std_msgs builtin_interfaces rosidl_default_generators \
      rosidl_typesupport_c nav2_bringup nav2_costmap_2d tf2_ros"
missing=""
for p in $PKGS; do
  [ -d "$ROS_PREFIX/share/$p" ] || missing="$missing $p"
done
if [ -z "$missing" ]; then
  pkg_list=$(echo $PKGS | tr -s '[:space:]' ' ' | tr ' ' ',')
  ok "ROS 包齐全（${pkg_list%,}）"
else
  for p in $missing; do
    case "$p" in
      nav2_*|tf2_ros) bad "缺 ROS 包 $p（导航栈用）"; hint "sudo apt install ros-$DISTRO-${p//_/-}" ;;
      *)              bad "缺 ROS 包 $p（驱动编译用）"; hint "sudo apt install ros-$DISTRO-${p//_/-}" ;;
    esac
  done
fi

echo
echo "[2] 编译工具链与三方库"
for t in colcon cmake g++ python3; do
  command -v "$t" >/dev/null && ok "$t 在" || { bad "$t 缺失"; hint "sudo apt install python3-colcon-common-extensions cmake g++"; }
done

if [ -d /usr/include/yaml-cpp ] || pkg-config --exists yaml-cpp 2>/dev/null; then
  ok "yaml-cpp 开发头文件在"
else
  bad "缺 yaml-cpp 开发包（驱动 CMakeLists 里 find_package(yaml-cpp REQUIRED) 会直接失败）"
  hint "sudo apt install libyaml-cpp-dev"
fi
if [ -f /usr/include/boost/thread.hpp ] || [ -f /usr/include/boost/thread/thread.hpp ]; then
  ok "boost thread 开发头文件在"
else
  bad "缺 boost thread 开发包"
  hint "sudo apt install libboost-thread-dev libboost-system-dev"
fi
python3 -c "import numpy" 2>/dev/null && ok "python3 numpy 在（干跑点云/中继脚本用）" \
  || { warn "python3 numpy 缺失（只影响 jt128_fake_cloud.py / jt128_sim_inject.py）"; hint "sudo apt install python3-numpy"; }

echo
echo "[3] 运行时工具"
for t in tcpdump nmcli rsync; do
  command -v "$t" >/dev/null && ok "$t 在" || warn "$t 缺失（可选）"
done
command -v ethtool >/dev/null && ok "ethtool 在" || warn "ethtool 缺失（只影响链路速率显示，setup_host.sh 已兼容）"

echo
echo "[3b] 上位机 LidarUtilities（可选；x86-64 的 GUI）"
if [ "$(uname -m)" = "x86_64" ]; then
  miss_lib=""
  for lib in libxcb-cursor.so.0 libxkbcommon-x11.so.0 libxcb-xinerama.so.0 libGL.so.1 libEGL.so.1 libdbus-1.so.3; do
    have_lib "$lib" || miss_lib="$miss_lib $lib"
  done
  if [ -z "$miss_lib" ]; then
    ok "Qt6 运行库齐全 —— LidarUtilities 可以在本机跑"
  else
    warn "缺 Qt6 运行库：$miss_lib"
    hint "sudo apt install libxcb-cursor0 libxkbcommon-x11-0 libxcb-xinerama0 libgl1 libegl1 libdbus-1-3"
  fi
  [ -n "$(ls "$HOME/jt128"/app/*.out 2>/dev/null)" ] && ok "已找到 app/ 下的 LidarUtilities" \
    || info "还没放 app/（把 .out 拷到 ~/jt128/app/ 即可，或用 port_setup.sh 的提示）"
else
  warn "本机架构 $(uname -m)：LidarUtilities（x86-64 二进制）跑不了，雷达参数请在 x86 机器上改"
fi

echo
echo "[4] 现有工作区"
COD_WS="$HOME/cod_-rm2026_-navigation"
if [ -d "$COD_WS/install" ]; then
  ok "cod 工作区已编译（$COD_WS）"
elif [ -d "$COD_WS" ]; then
  warn "cod 工作区存在但没编译（缺 install/）→ cd $COD_WS && colcon build"
else
  bad "找不到 cod 工作区（真机导航栈要用）"
fi
[ -d "$COD_WS/install/cpp_lidar_filter" ] \
  && ok "cpp_lidar_filter 已编译（真机路线的滤波节点）" \
  || warn "cpp_lidar_filter 没编译（真机路线需要）"

# 仿真能力：这些包由 cod 仓库的 src/fzsd_vendor/ 提供，**不需要**外部 fzsd2025 工作区
echo
echo "[4b] 仿真能力（可选，只有跑 Gazebo HIL 才需要）"
sim_missing=""
for p in rmu_gazebo_simulator fzsd2025_robot_description pb2025_nav_bringup sdformat_tools cod_sim; do
  [ -d "$ROS_PREFIX/share/$p" ] || [ -d "$COD_WS/install/$p" ] || sim_missing="$sim_missing $p"
done
if [ -z "$sim_missing" ]; then
  ok "仿真包齐全（rmu_gazebo_simulator / fzsd2025_robot_description / pb2025_nav_bringup / sdformat_tools / cod_sim）"
  command -v gz >/dev/null 2>&1 || command -v ign >/dev/null 2>&1 \
    && ok "Gazebo 命令在" || warn "找不到 gz/ign 命令（仿真起不来，需要 Gazebo）"
  nvidia-smi >/dev/null 2>&1 && ok "GPU 可用（Gazebo 建议有独显）" || warn "没检测到 NVIDIA GPU（Gazebo 可能很慢）"
else
  warn "缺仿真包：$sim_missing（不跑仿真就不影响；cod 里没编译 fzsd_vendor 的话 cd $COD_WS && colcon build）"
fi
[ -d "$HOME/fzsd2025" ] && echo "     （注：~/fzsd2025 存在，但仿真并不需要它，仿真包在 cod 里）"

echo
echo "[5] DDS 与网络"
rmw="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp(默认)}"
ok "RMW: $rmw"
if [ "$(uname -m)" != "x86_64" ]; then
  warn "架构是 $(uname -m)：上位机 LidarUtilities（x86-64 的 .out）在本机跑不了，改参数请回到 x86 桌面机"
fi
rmem=$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)
if [ "$rmem" -ge 4194304 ] 2>/dev/null; then
  ok "net.core.rmem_max = $rmem（够大）"
else
  warn "net.core.rmem_max = $rmem（20 万字节级）—— 1.84 MB/帧的点云全靠 Fast DDS 共享内存段，"
  hint "脚本用的 fastdds_large_msg.xml（64 MB 段）就是为了绕开这个限制，确保它被使用"
fi

echo
echo "======================================================"
printf "结果：\033[32m%d 通过\033[0m，\033[33m%d 警告\033[0m，\033[31m%d 失败\033[0m\n" "$OK" "$WARN" "$FAIL"
[ "$FAIL" -eq 0 ] && echo "→ 环境就绪，可以跑 port_setup.sh 编译驱动" || echo "→ 先把失败项装上再继续"
exit "$FAIL"
