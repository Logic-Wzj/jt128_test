#!/usr/bin/env bash
# 打一个可移植的 JT128 移植包（在你这台桌面机上跑）
#
# 用法：bash make_bundle.sh [输出目录]
# 产物：jt128_port_<日期>.tar.gz  —— 拷到机器人主机解压后跑 port/port_setup.sh
#
# 只打包"源码 + 脚本 + 文档"，**不打 build/install**：
#   - install/ 里是软链（指向 /home/<user>/hesai_ws/build/...），换机器必然断链
#   - CMakeCache 里嵌了 11 处本机绝对路径，换机器必须重新编译
# 上位机 LidarUtilities（100 MB，x86-64）默认不打进去；要用 -a 显式加上。

set -eu

case "${1:-}" in
  -h|--help|help)
    # 打印文件开头的注释块（到第一个非注释行为止）
    awk 'NR>1 && /^#/ { sub(/^# ?/, ""); print; next } NR>1 { exit }' "$0"
    echo
    echo "用法：bash port/make_bundle.sh [输出目录] [-a]"
    echo "  [输出目录]  默认 \$HOME"
    echo "  -a          额外打包上位机 LidarUtilities（~100MB，x86-64；需先放到 jt128/app/）"
    exit 0 ;;
esac

OUT_DIR="${1:-$HOME}"
# 参数若像选项而不是目录，直接报错，别把 '--xxx' 当目录用
case "$OUT_DIR" in
  -*) echo "参数看起来是选项而不是目录：$OUT_DIR（用法见 --help）"; exit 1 ;;
esac
STAMP=$(date +%Y%m%d)
NAME="jt128_port_$STAMP"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

SRC_JT128="$HOME/jt128"
SRC_DRIVER="$HOME/HesaiLidar_ROS_2.0"
SRC_RELAY="$HOME/hesai_ws/src/jt128_sim_relay"
SRC_COMPAT="$HOME/hesai_ws/src/jt128_livox_compat"   # JT128 到 Livox 的适配层（真机接入用）
WITH_APP=0
[ "${2:-}" = "-a" ] && WITH_APP=1

for d in "$SRC_JT128" "$SRC_DRIVER" "$SRC_RELAY" "$SRC_COMPAT"; do
  [ -e "$d" ] || { echo "缺 $d"; exit 1; }
done

mkdir -p "$TMP/$NAME"

echo "== 1) 脚本与文档 =="
rsync -a --exclude '__pycache__' --exclude 'app/' --exclude '*.pyc' \
  "$SRC_JT128/" "$TMP/$NAME/jt128/"
[ "$WITH_APP" = 1 ] && [ -d "$SRC_JT128/app" ] && rsync -a "$SRC_JT128/app/" "$TMP/$NAME/jt128/app/"

echo "== 2) 禾赛驱动源码（含子模块，去掉 .git 记录版本号）=="
rsync -a --exclude '.git' --exclude 'build' --exclude 'install' --exclude 'log' \
  --exclude '__pycache__' --exclude '*.pyc' \
  "$SRC_DRIVER/" "$TMP/$NAME/HesaiLidar_ROS_2.0/"

echo "== 3) C++ 中继包 =="
rsync -a --exclude 'build' --exclude 'install' --exclude 'log' \
  --exclude '__pycache__' --exclude '*.pyc' \
  "$SRC_RELAY/" "$TMP/$NAME/jt128_sim_relay/"

echo "== 3b) JT128->Livox 适配层 =="
rsync -a --exclude 'build' --exclude 'install' --exclude 'log' \
  --exclude '__pycache__' --exclude '*.pyc' \
  "$SRC_COMPAT/" "$TMP/$NAME/jt128_livox_compat/"

echo "== 4) 记录版本信息 =="
{
  echo "打包时间: $(date '+%F %T %z')"
  echo "打包机器: $(hostname) / $(uname -m) / $(lsb_release -ds 2>/dev/null)"
  echo
  echo "---- 禾赛驱动 ----"
  ( cd "$SRC_DRIVER" && git log --oneline -1 && git describe --tags 2>/dev/null && echo "submodule:" && git submodule status )
  echo
  echo "---- 中继包 ----"
  echo "jt128_sim_relay v0.1.0（依赖 rclcpp + sensor_msgs）"
  echo
  echo "---- 目标机步骤 ----"
  echo "1. tar xzf 本包"
  echo "2. bash jt128/port/deps_check.sh"
  echo "3. bash jt128/port/port_setup.sh \$PWD"
} > "$TMP/$NAME/VERSION.txt"
cat "$TMP/$NAME/VERSION.txt"

echo "== 5) 打包 =="
tar -czf "$OUT_DIR/$NAME.tar.gz" -C "$TMP" "$NAME"
echo
echo "产物：$OUT_DIR/$NAME.tar.gz  ($(du -h "$OUT_DIR/$NAME.tar.gz" | cut -f1))"
tar -tzf "$OUT_DIR/$NAME.tar.gz" | sed -n '1,12p' | sed 's/^/    /'
echo "    ... 共 $(tar -tzf "$OUT_DIR/$NAME.tar.gz" | wc -l) 个条目"
[ "$WITH_APP" = 0 ] && echo
[ "$WITH_APP" = 0 ] && echo "提示：上位机 LidarUtilities（x86-64）没打进去。需要的话：bash make_bundle.sh $OUT_DIR -a"
