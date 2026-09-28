#!/usr/bin/env bash
# 在本机排演"小电脑从 GitHub 拉下来并部署"的全过程，全程隔离，不碰你现有的
# ~/jt128、~/HesaiLidar_ROS_2.0、~/hesai_ws（全部落在临时 HOME 里）。
#
# 用法:
#   ./verify_transfer.sh                            # 用默认仓库地址排演
#   ./verify_transfer.sh <仓库URL>                   # 指定仓库/分支
#   ./verify_transfer.sh --clean                    # 排演完删掉沙箱（默认保留供检查）
#   ./verify_transfer.sh https://... --keep         # 公开仓库走 HTTPS
#
# 环境变量:
#   JT128_VERIFY_DIR   沙箱目录（默认 /tmp/jt128_verify）
#   JT128_REPO_URL     仓库地址（默认 Logic-Wzj/jt128_test 的 SSH 地址）
#   JT128_REPO_BRANCH  分支（默认 main）
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

REPO_URL="${JT128_REPO_URL:-git@github.com:Logic-Wzj/jt128_test.git}"
BRANCH="${JT128_REPO_BRANCH:-main}"
SB="${JT128_VERIFY_DIR:-/tmp/jt128_verify}"
CLEAN=0

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help)   sed -n '2,16p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    --clean)     CLEAN=1; shift ;;
    --keep)      CLEAN=0; shift ;;
    -*)          echo "未知参数：$1（-h 看用法）" >&2; exit 2 ;;
    *)           REPO_URL="$1"; shift ;;
  esac
done

# 安全阀：沙箱必须是个明确的临时目录
case "$SB" in
  /tmp/*|"$HOME"/*) ;;
  *) echo "拒绝：沙箱目录看起来不安全：$SB（用 JT128_VERIFY_DIR 指到 /tmp 或 \$HOME 下）" >&2; exit 2 ;;
esac
[ "$SB" = "/" ] || [ "$SB" = "$HOME" ] && { echo "拒绝：沙箱目录不能是 $SB" >&2; exit 2; }

say()  { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
info() { printf '   %s\n' "$*"; }
ok()   { printf '\033[32m   ✅ %s\033[0m\n' "$*"; PASS=$((PASS+1)); }
bad()  { printf '\033[31m   ❌ %s\033[0m\n' "$*"; FAIL=$((FAIL+1)); }
warn() { printf '\033[33m   ⚠️  %s\033[0m\n' "$*"; }
PASS=0; FAIL=0

printf '\033[1mJT128 搬运流程本机排演\033[0m\n'
info "仓库    : $REPO_URL （分支 $BRANCH）"
info "沙箱    : $SB  （你的 ~/jt128、~/hesai_ws 不会被碰）"

# ---------- 1. 克隆 ----------
say "1/7 从 GitHub 克隆（模拟小电脑）"
rm -rf "$SB"
mkdir -p "$SB" || { bad "建不了沙箱目录 $SB"; exit 1; }
CLONE_DIR="$SB/jt128_test"
if timeout 180 git clone -q -b "$BRANCH" "$REPO_URL" "$CLONE_DIR" 2>"$SB/clone.err"; then
  ok "克隆成功（$(du -sh "$CLONE_DIR" | cut -f1)）"
else
  bad "克隆失败：$(head -c 200 "$SB/clone.err")"
  warn "私有仓库需要认证：本机 SSH key 已注册，确认地址是 git@github.com:... 开头"
  exit 1
fi

# ---------- 2. 内容完整性 ----------
say "2/7 仓库内容"
NFILES=$(cd "$CLONE_DIR" && git ls-files | wc -l)
[ "$NFILES" -ge 270 ] && ok "$NFILES 个文件" || bad "文件数异常：$NFILES（预期 ≥270）"
for d in jt128 jt128_sim_relay jt128_livox_compat HesaiLidar_ROS_2.0 README.md THIRD_PARTY.md; do
  [ -e "$CLONE_DIR/$d" ] && ok "有 $d" || bad "缺 $d"
done
[ -d "$CLONE_DIR/HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0/libhesai" ] \
  && ok "驱动自带 SDK 源码（不联网也能编译）" || bad "驱动缺 SDK 源码"
[ -x "$CLONE_DIR/jt128/launch.sh" ] && ok "脚本带可执行位" \
  || warn "脚本没有可执行位（ZIP 下载常见），用 bash 调用即可"

# ---------- 3. 依赖自检 ----------
say "3/7 依赖自检（deps_check.sh）"
HOME="$SB" bash "$CLONE_DIR/jt128/port/deps_check.sh" >"$SB/deps.log" 2>&1
DEPS_LINE=$(grep -E "结果：" "$SB/deps.log" | tail -1 | sed 's/\x1b\[[0-9;]*m//g')
info "${DEPS_LINE:-（没拿到结果行，见 $SB/deps.log）}"
if grep -q "❌" "$SB/deps.log"; then
  warn "有失败项——本机沙箱里缺 cod 工作区属正常（小电脑上有）；其它失败项要处理"
else
  ok "无失败项"
fi

# ---------- 4. 部署 ----------
say "4/7 部署（port_setup.sh，全部装进沙箱 HOME）"
if HOME="$SB" timeout 600 bash "$CLONE_DIR/jt128/port/port_setup.sh" "$CLONE_DIR" >"$SB/setup.log" 2>&1; then
  ok "部署脚本退出码 0"
else
  bad "部署失败，看 $SB/setup.log"
  tail -15 "$SB/setup.log" | sed 's/^/     /'
fi

# ---------- 5. 产物检查 ----------
say "5/7 部署产物"
[ -f "$SB/jt128/launch.sh" ] && ok "~/jt128 已安装" || bad "~/jt128 没装上"
for p in hesai_ros_driver jt128_sim_relay jt128_livox_compat; do
  [ -d "$SB/hesai_ws/install/$p" ] && ok "编译产物 $p" || bad "缺编译产物 $p"
done
[ -f "$SB/HesaiLidar_ROS_2.0/config/config.yaml" ] && ok "驱动源码就位" || bad "驱动源码没就位"

# ---------- 6. 配置项 ----------
say "6/7 驱动 config.yaml 是否被改对"
CFG="$SB/HesaiLidar_ROS_2.0/config/config.yaml"
check_cfg() {  # $1=字段 $2=期望值
  v="$(grep -m1 -E "^[[:space:]]*$1:" "$CFG" 2>/dev/null | sed 's/#.*//' | cut -d: -f2- \
       | sed 's/^[[:space:]]*//; s/[[:space:]]*$//' | tr -d '"')"
  if [ "$v" = "$2" ]; then ok "$1 = $v"; else bad "$1 = '${v:-空}'（期望 $2）"; fi
}
check_cfg ros_frame_id front_jt128
check_cfg ptc_connect_timeout 3
check_cfg multicast_ip_address ""
check_cfg channel_fov_filter_path ""
for key in correction_file_path firetimes_path; do
  p="$(grep -m1 -E "^[[:space:]]*$key:" "$CFG" | sed 's/#.*//' | sed 's/^[^:]*:[[:space:]]*//' \
       | sed 's/^[[:space:]]*//; s/[[:space:]]*$//' | tr -d '"')"
  if [ -f "$p" ] && [ "${p#"$SB"}" != "$p" ]; then ok "$key 指向沙箱内的真实文件"
  else bad "$key 路径不对或文件不存在：${p:-空}"; fi
done

# ---------- 7. 脚本能跑 ----------
say "7/7 脚本自检"
if HOME="$SB" bash "$SB/jt128/launch.sh" help >"$SB/help.log" 2>&1 && grep -q "radar" "$SB/help.log"; then
  ok "launch.sh 可用（$(grep -cE '^  [a-z]+\)' "$SB/jt128/launch.sh") 个子命令）"
else
  bad "launch.sh help 异常，看 $SB/help.log"
fi
if HOME="$SB" python3 "$SB/jt128/jt128_check.py" --selftest >"$SB/selftest.log" 2>&1; then
  ok "jt128_check.py 自检通过（裸 UDP 解析器正常）"
else
  bad "jt128_check.py --selftest 失败，看 $SB/selftest.log"
fi
[ -f "$SB/.bashrc" ] && grep -q "jt128" "$SB/.bashrc" && ok "shell 入口已写入沙箱 .bashrc" \
  || warn "沙箱 .bashrc 里没看到 jt128 入口"

# ---------- 汇总 ----------
say "结果：$PASS 项通过，$FAIL 项失败"
if [ "$FAIL" = 0 ]; then
  printf '\033[32m   本机排演全部通过 → 小电脑上按下面 4 条走就行\033[0m\n'
else
  printf '\033[31m   有失败项，先处理再搬（日志都在 %s/）\033[0m\n' "$SB"
fi
cat <<EOF

   小电脑上：
     git clone -b $BRANCH $REPO_URL jt128_test
     cd jt128_test
     ./jt128/port/deps_check.sh
     ./jt128/port/port_setup.sh "\$PWD"
     source ~/.bashrc && ./jt128/launch.sh radar

   排演沙箱：$SB
     $SB/jt128_test          克隆下来的仓库
     $SB/jt128               部署出来的脚本
     $SB/hesai_ws            编译出来的工作区
     $SB/*.log               各步日志
EOF
[ "$CLEAN" = 1 ] && { rm -rf "$SB"; echo "   已清理沙箱"; }
exit $([ "$FAIL" = 0 ] && echo 0 || echo 1)
