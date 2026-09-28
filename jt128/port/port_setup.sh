#!/usr/bin/env bash
# JT128 机器人主机一键部署（在**目标机**上跑，解包之后执行）
#
# 用法：
#     tar xzf jt128_port_YYYYMMDD.tar.gz
#     cd jt128_port_YYYYMMDD
#     bash jt128/port/port_setup.sh "$PWD"
#
# 它做的事：
#   1. 环境自检（deps_check.sh）
#   2. 把 /home/<本机用户>/jt128 建好（脚本+文档）
#   3. 禾赛驱动装到 ~/HesaiLidar_ROS_2.0，中继包装到 ~/hesai_ws/src/
#   4. **改写 config.yaml 里两处硬编码绝对路径**（correction / firetimes）→ 换成目标机实际路径
#   5. colcon build 驱动 + 中继
#   6. 往 ~/.zshrc 追加 source 行（先备份，幂等）

set -u

BUNDLE="${1:-$PWD}"
JT128_SRC="$BUNDLE/jt128"
DRIVER_SRC="$BUNDLE/HesaiLidar_ROS_2.0"
RELAY_SRC="$BUNDLE/jt128_sim_relay"
COMPAT_SRC="$BUNDLE/jt128_livox_compat"
DEST_JT128="${JT128_DEST:-$HOME/jt128}"   # 可用 JT128_DEST 覆盖安装目录名
DEST_DRIVER="$HOME/HesaiLidar_ROS_2.0"
DEST_WS="$HOME/hesai_ws"

say()  { printf "\n\033[1m== %s ==\033[0m\n" "$1"; }
warn() { printf "\033[33m⚠️  %s\033[0m\n" "$1"; }
die()  { printf "\033[31m❌ %s\033[0m\n" "$1"; exit 1; }

[ -d "$JT128_SRC" ] || die "找不到 $JT128_SRC（用法：bash port_setup.sh <解包目录>）"
# 驱动源码：包里没有就不强制（下面会改为从 GitHub 克隆）

say "1/6 环境自检"
bash "$JT128_SRC/port/deps_check.sh" || warn "自检有失败项，编译可能会失败（继续执行，稍后看报错）"

say "2/6 安装脚本到 $DEST_JT128"
if [ -d "$DEST_JT128" ]; then
  warn "$DEST_JT128 已存在，跳过（要覆盖就自己删掉再跑）"
else
  cp -r "$JT128_SRC" "$DEST_JT128"
  chmod +x "$DEST_JT128"/*.sh "$DEST_JT128"/*.py "$DEST_JT128"/port/*.sh 2>/dev/null
  echo "已安装"
fi

say "3/6 安装驱动源码到 $DEST_DRIVER"

# 驱动能不能编译，取决于子模块 HesaiLidar_SDK_2.0 是否在位。
# 克隆中断会留下"半成品"目录（有外层、没 SDK），所以这里统一校验+修复。
SDK_DIR="$DEST_DRIVER/src/driver/HesaiLidar_SDK_2.0"
sdk_ok() { [ -d "$SDK_DIR/libhesai" ] && [ -d "$SDK_DIR/driver" ]; }
fetch_submodules() {  # 带重试地拉子模块
  local i
  for i in 1 2 3; do
    ( cd "$DEST_DRIVER" && git submodule update --init --recursive ) && sdk_ok && return 0
    warn "子模块拉取第 $i 次失败，重试…"
    sleep 5
  done
  return 1
}

if [ -d "$DEST_DRIVER" ]; then
  if sdk_ok; then
    warn "$DEST_DRIVER 已存在，跳过（避免覆盖你的改动）"
  else
    warn "$DEST_DRIVER 已存在但缺 SDK 子模块（上次克隆中断？），尝试补齐"
    if fetch_submodules; then
      ok "子模块已补齐"
    else
      die "补齐失败：删掉 $DEST_DRIVER 重跑，或手动 git submodule update --init --recursive
      网络不稳可换源：HESAI_DRIVER_URL=<镜像地址> $0"
    fi
  fi
elif [ -d "$DRIVER_SRC" ]; then
  cp -r "$DRIVER_SRC" "$DEST_DRIVER"
  echo "已从移植包安装驱动源码"
  sdk_ok || warn "移植包里的驱动缺 SDK 子模块，编译可能失败（重新执行 make_bundle.sh -a 打包）"
else
  # 包里没带驱动源码（GitHub 轻量分发方式）：直接从上游克隆
  local_url="${HESAI_DRIVER_URL:-https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git}"
  # 默认固定到实测过的 tag；想跟上游 HEAD 就 HESAI_DRIVER_REF= 置空
  ref="${HESAI_DRIVER_REF-v2.0.12}"
  warn "移植包里没有驱动源码，改为从上游克隆：$local_url${ref:+（版本 $ref）}"
  command -v git >/dev/null 2>&1 || die "没装 git，无法克隆；请改用带驱动源码的完整移植包"
  # GitHub 偶发抽风（外层仓库成功、子模块失败），所以重试并在每次前清掉半成品
  cloned=0
  for i in 1 2 3; do
    rm -rf "$DEST_DRIVER"
    if git clone "$local_url" "$DEST_DRIVER"; then cloned=1; break; fi
    warn "克隆第 $i 次失败，重试…"; sleep 5
  done
  [ "$cloned" = 1 ] || die "git clone 连续失败。可换源：HESAI_DRIVER_URL=<镜像/本地路径> $0"
  if [ -n "$ref" ]; then
    ( cd "$DEST_DRIVER" && git checkout "$ref" ) \
      || warn "切到 $ref 失败，继续用默认分支（功能通常不受影响）"
  fi
  fetch_submodules || die "驱动外层已克隆，但 SDK 子模块拉不下来（网络不稳/被墙）。可换源重试：
       HESAI_DRIVER_URL=<镜像地址> $0
     或改用带驱动源码的完整移植包（make_bundle.sh -a）"
  echo "已从上游克隆（BSD-3 许可，LICENSE 随仓库保留）"
fi

say "4/6 修正 config.yaml（本机绝对路径 + JT128 专用项）"
CFG="$DEST_DRIVER/config/config.yaml"
[ -f "$CFG" ] || die "找不到 $CFG"
# 统一走 launch.sh fixpaths（唯一实现）：改角度/开火时间文件路径，
# 并把 ros_frame_id / ptc_connect_timeout / multicast / fov 过滤等 JT128 专用项设对。
# 上游原版配置里 ptc_connect_timeout=-1 会让驱动在 PTC 不通时无限阻塞，必须改。
HESAI_REPO="$DEST_DRIVER" HESAI_WS="$DEST_WS" bash "$DEST_JT128/launch.sh" fixpaths \
  || die "修正 $CFG 失败（可手动执行：HESAI_REPO=$DEST_DRIVER bash $DEST_JT128/launch.sh fixpaths）"

# 解码要用到的两个修正文件必须真的在
for f in "correction/angle_correction/JT128_Angle Correction File.csv" \
         "correction/firetime_correction/JT128_Firetime Correction File.csv"; do
  p="$DEST_DRIVER/src/driver/HesaiLidar_SDK_2.0/$f"
  if [ -f "$p" ]; then
    echo "  文件存在 ✓ $(basename "$f")"
  else
    warn "缺少 $(basename "$f")：$p（点云角度/时间会有偏差）"
  fi
done

say "5/6 编译驱动 + 中继"
mkdir -p "$DEST_WS/src"
[ -e "$DEST_WS/src/HesaiLidar_ROS_2.0" ] || ln -s "$DEST_DRIVER" "$DEST_WS/src/HesaiLidar_ROS_2.0"
if [ -e "$DEST_WS/src/jt128_sim_relay" ]; then
  warn "$DEST_WS/src/jt128_sim_relay 已存在，跳过拷贝"
else
  cp -r "$RELAY_SRC" "$DEST_WS/src/jt128_sim_relay"
fi
if [ -d "$COMPAT_SRC" ]; then
  if [ -e "$DEST_WS/src/jt128_livox_compat" ]; then
    warn "$DEST_WS/src/jt128_livox_compat 已存在，跳过拷贝"
  else
    cp -r "$COMPAT_SRC" "$DEST_WS/src/jt128_livox_compat"
  fi
fi

set +u
DISTRO="${ROS_DISTRO:-}"
if [ -z "$DISTRO" ]; then for d in /opt/ros/*; do [ -d "$d" ] && DISTRO=$(basename "$d") && break; done; fi
DISTRO="${DISTRO:-humble}"
[ -f "/opt/ros/$DISTRO/setup.bash" ] || die "找不到 /opt/ros/$DISTRO/setup.bash"
source "/opt/ros/$DISTRO/setup.bash"
cd "$DEST_WS"
colcon build --symlink-install || die "编译失败，看上面的报错（多半是缺依赖，重跑 deps_check.sh）"
set -u
echo "编译完成：$DEST_WS/install"

say "6/6 配置 shell 入口（zsh 和 bash 都处理）"
# zsh：source jt128.zsh（提供 jt128* 系列函数）
if [ -f "$HOME/.zshrc" ]; then
  if grep -q "jt128.zsh" "$HOME/.zshrc" 2>/dev/null; then
    warn ".zshrc 里已有 source 行，跳过"
  else
    cp "$HOME/.zshrc" "$HOME/.zshrc.bak-jt128-$(date +%s)" 2>/dev/null || true
    cat >> "$HOME/.zshrc" <<'EOF'

# JT128 雷达测试环境（port_setup.sh 追加；删掉这三行即可移除）
if [ -f "'$DEST_JT128'/jt128.zsh" ]; then
  source "'$DEST_JT128'/jt128.zsh"
fi
EOF
    echo "已追加到 ~/.zshrc（原文件已备份为 ~/.zshrc.bak-jt128-*）"
  fi
else
  warn "没有 ~/.zshrc（这台机器可能没装 zsh）"
fi

# bash：加一个 jt128 别名指向 launch.sh（launch.sh 是纯 bash，不依赖 zsh）
if [ -f "$HOME/.bashrc" ]; then
  if grep -q "jt128.*launch.sh" "$HOME/.bashrc" 2>/dev/null; then
    warn ".bashrc 里已有 jt128 别名，跳过"
  else
    cp "$HOME/.bashrc" "$HOME/.bashrc.bak-jt128-$(date +%s)" 2>/dev/null || true
    cat >> "$HOME/.bashrc" <<'EOF'

# JT128 雷达测试入口（port_setup.sh 追加；删掉这一行即可移除）
alias jt128='$DEST_JT128/launch.sh'
EOF
    echo "已追加到 ~/.bashrc（重开终端后可用 jt128 help）"
  fi
fi

cat <<EOF

==========================================================
部署完成。接下来在**新开的终端**里：

  1. 给雷达网口配 IP（把 enp131s0 换成目标机实际网卡名）：
       sudo ~/jt128/setup_host.sh <网卡名> 192.168.1.100
       # 想持久化：sudo nmcli con add type ethernet ifname <网卡名> con-name jt128 ip4 192.168.1.100/24

  2. 先证明雷达在发数据（不依赖 ROS）：
       jt128check

  3. 接真机导航栈：
       jt128up && jt128verify
     接仿真 HIL（仿真包已随 cod 仓库编译，无需外部 fzsd2025）：
       jt128sim   # 另一个终端
       jt128hil

  注意：本包不含上位机 LidarUtilities（x86-64 二进制），ARM 主机上跑不了；
       要改雷达参数请回到 x86 桌面机。
==========================================================
EOF
