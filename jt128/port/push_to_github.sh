#!/usr/bin/env bash
# 把 JT128 现场测试工具包整理成一个 git 仓库并（可选）推到远端。
#
# 默认**不**包含禾赛驱动源码：小电脑部署时由 port_setup.sh 自动从上游 BSD-3 仓库克隆
# （固定 tag v2.0.12），这样仓库只有几百 KB，也避免再分发第三方代码。
# 需要连驱动一起带上就用 --with-driver（建议私有仓库）。
#
# 用法:
#   ./push_to_github.sh                      # 只整理到本地仓库，并打印后续命令
#   ./push_to_github.sh --with-driver        # 连驱动源码一起放进去
#   ./push_to_github.sh --dir ~/jt128_repo   # 指定输出目录
#   ./push_to_github.sh git@github.com:me/jt128-test-kit.git   # 整理并推送
#
# 环境变量:
#   JT128_GIT_REMOTE   等价于命令行传远端 URL
#   JT128_GIT_BRANCH   默认 main
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JT128_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
HESAI_WS="${HESAI_WS:-$HOME/hesai_ws}"

WITH_DRIVER=0
OUT_DIR="${JT128_GIT_DIR:-$HOME/jt128_git_export}"
REMOTE="${JT128_GIT_REMOTE:-}"
BRANCH="${JT128_GIT_BRANCH:-main}"

say()  { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
info() { printf '   %s\n' "$*"; }
ok()   { printf '\033[32m   ✅ %s\033[0m\n' "$*"; }
warn() { printf '\033[33m   ⚠️  %s\033[0m\n' "$*"; }
err()  { printf '\033[31m   ❌ %s\033[0m\n' "$*" >&2; }
die()  { err "$*"; exit 1; }

usage() { sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0; }

# ---------- 参数 ----------
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help)      usage ;;
    --with-driver)  WITH_DRIVER=1; shift ;;
    --dir)          [ -n "${2:-}" ] || die "--dir 需要目录参数"; OUT_DIR="$2"; shift 2 ;;
    --dir=*)        OUT_DIR="${1#*=}"; shift ;;
    -*)             die "未知参数：$1（-h 看用法）" ;;
    *)              REMOTE="$1"; shift ;;
  esac
done

# ---------- 找源码位置（工作台 或 移植包布局） ----------
find_pkg() {  # $1=包名
  for c in "$HESAI_WS/src/$1" "$JT128_DIR/../$1" "$JT128_DIR/$1"; do
    [ -d "$c" ] && { printf '%s\n' "$(cd "$c" && pwd)"; return 0; }
  done
  return 1
}
RELAY_SRC="$(find_pkg jt128_sim_relay || true)"
COMPAT_SRC="$(find_pkg jt128_livox_compat || true)"
DRIVER_SRC="${HESAI_REPO:-$HOME/HesaiLidar_ROS_2.0}"
[ -d "$DRIVER_SRC" ] || DRIVER_SRC="$JT128_DIR/../HesaiLidar_ROS_2.0"

command -v git >/dev/null 2>&1 || die "没装 git"

say "整理内容"
info "工具包   : $JT128_DIR"
info "中继包   : ${RELAY_SRC:-（没找到，跳过）}"
info "兼容层   : ${COMPAT_SRC:-（没找到，跳过）}"
if [ "$WITH_DRIVER" = 1 ]; then
  [ -d "$DRIVER_SRC" ] || die "--with-driver 但没找到驱动源码（HESAI_REPO=$DRIVER_SRC）"
  info "驱动源码 : $DRIVER_SRC（随包分发）"
else
  info "驱动源码 : 不打包（小电脑部署时从上游 BSD-3 仓库自动克隆 v2.0.12）"
fi
info "输出目录 : $OUT_DIR"

# ---------- 复制（排除产物） ----------
# 保留已有 .git：这样改完脚本重跑一次就能直接 git push（否则历史不同源，会被要求 --force）
GITKEEP=""
if [ -d "$OUT_DIR/.git" ]; then
  GITKEEP="$(mktemp -d)"
  mv "$OUT_DIR/.git" "$GITKEEP/.git" || die "无法暂存 $OUT_DIR/.git"
  info "检测到已有 git 历史，保留（重跑后可直接 push）"
fi
rm -rf "$OUT_DIR" || die "清不掉 $OUT_DIR"
mkdir -p "$OUT_DIR"
if [ -n "$GITKEEP" ]; then mv "$GITKEEP/.git" "$OUT_DIR/.git" && rmdir "$GITKEEP"; fi

EXCLUDES=(--exclude=.git --exclude=__pycache__ --exclude='*.pyc' --exclude=app
          --exclude=build --exclude=install --exclude=log --exclude='*.npy'
          --exclude='*.log' --exclude='*.tar.gz')
copy_tree() {  # $1=源 $2=目标相对路径
  local src="$1" dst="$OUT_DIR/$2"
  mkdir -p "$(dirname "$dst")"
  if command -v rsync >/dev/null 2>&1; then
    rsync -a "${EXCLUDES[@]}" "$src/" "$dst/"
  else
    cp -r "$src" "$dst"
    find "$dst" -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
    find "$dst" -name '*.pyc' -delete 2>/dev/null
    rm -rf "$dst/app" "$dst/build" "$dst/install" "$dst/log" "$dst/.git"
  fi
}

copy_tree "$JT128_DIR" jt128
[ -n "$RELAY_SRC" ]  && copy_tree "$RELAY_SRC"  jt128_sim_relay
[ -n "$COMPAT_SRC" ] && copy_tree "$COMPAT_SRC" jt128_livox_compat
if [ "$WITH_DRIVER" = 1 ]; then
  copy_tree "$DRIVER_SRC" HesaiLidar_ROS_2.0
  rm -rf "$OUT_DIR/HesaiLidar_ROS_2.0/.git"
  [ -d "$OUT_DIR/HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0/.git" ] && \
    rm -rf "$OUT_DIR/HesaiLidar_ROS_2.0/src/driver/HesaiLidar_SDK_2.0/.git"
fi
chmod +x "$OUT_DIR"/jt128/*.sh "$OUT_DIR"/jt128/port/*.sh 2>/dev/null

# ---------- 体积检查（GitHub 单文件 100 MB 硬限制） ----------
BIG="$(find "$OUT_DIR" -type f -size +10M -printf '%s\t%p\n' 2>/dev/null | sort -rn)"
if [ -n "$BIG" ]; then
  err "有超过 10 MB 的文件，GitHub 不适合放这些（尤其 LidarUtilities 上位机）："
  printf '%s\n' "$BIG" | while IFS=$'\t' read -r sz p; do info "$((sz/1048576)) MB  $p"; done
  die "请把这些文件排除（用 scp/U盘 单独拷），或改放 Release 附件"
fi
TOTAL="$(du -sh "$OUT_DIR" | cut -f1)"
ok "体积检查通过（合计 $TOTAL，无 >10 MB 文件）"

# ---------- 生成 README / 第三方说明 ----------
if [ "$WITH_DRIVER" = 1 ]; then
  DRIVER_NOTE="- \`HesaiLidar_ROS_2.0/\` 禾赛官方 ROS2 驱动（**Modified BSD-3**，LICENSE 随包保留，上游 commit \`e7e112f\` / tag \`v2.0.12\`）"
else
  DRIVER_NOTE="- 禾赛官方驱动**不在本仓库**：部署时 \`port_setup.sh\` 自动从 https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git 克隆（tag \`v2.0.12\`）"
fi
cat > "$OUT_DIR/README.md" <<EOF
# JT128 现场测试工具包

给"小电脑 / 机器人主机"用的禾赛 JT128 雷达测试与接入脚本（ROS 2 Humble / Ubuntu 22.04）。

## 小电脑三步

\`\`\`bash
git clone <本仓库地址> jt128_test && cd jt128_test   # 目录叫什么都行
./jt128/port/deps_check.sh                            # 1. 依赖自检（应无 ❌）
./jt128/port/port_setup.sh "\$PWD"                     # 2. 部署（装驱动+编译+写 shell 入口）
source ~/.bashrc && ./jt128/launch.sh radar            # 3. 配网+探包+启动驱动
\`\`\`

## 内容

- \`jt128/\` 工具套件：\`launch.sh\`（一键入口）、\`jt128_check.py\`（裸 UDP 收包自检）、\`memwatch.py\`（内存/CPU/FD 监控）、
  \`jt128_verify.sh\`（验收）、\`port/\`（移植部署）、\`sim/\`（仿真 HIL 用）
- \`jt128_sim_relay/\` 仿真中继（\`/lidar_points\` → \`/jt128/points\`）
- \`jt128_livox_compat/\` 禾赛点云 → Livox 格式兼容层（喂给 small_point_lio）

$DRIVER_NOTE

## 文档

**不随本仓库分发**：所有说明文档（上手步骤、接上雷达后、完整验收清单、HIL 启动方式、真机接入、GitHub 传包……）
保留在准备机的 `~/jt128_文档/` 目录里，需要时单独拷过去即可。本仓库只放**能跑的代码**。
EOF

cat > "$OUT_DIR/THIRD_PARTY.md" <<'EOF'
# 第三方组件与许可

本仓库只包含现场测试脚本与两个 ROS 2 包（中继 / Livox 兼容层）。

- **HesaiLidar_ROS_2.0**（禾赛官方 ROS2 驱动）与其子模块 **HesaiLidar_SDK_2.0**：
  Modified BSD-3-Clause，版权归 Hesai Technology。
  - 轻量方式：不在本仓库内，由 `jt128/port/port_setup.sh` 在部署时从
    https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git 克隆（tag `v2.0.12`），
    上游 LICENSE 随克隆保留，未做再分发。
  - `--with-driver` 方式：仓库内含驱动源码，请**保留全部 LICENSE 文件**，并遵守
    Modified BSD-3（保留版权声明与免责声明）。
- **LidarUtilities_Sample_JT128（上位机 GUI）**：禾赛专有二进制，**不在本仓库**，
  请勿提交；需要改雷达参数时用 scp/U 盘单独拷到 x86 机器。
EOF

cat > "$OUT_DIR/.gitignore" <<'EOF'
__pycache__/
*.pyc
*.npy
*.log
*.tar.gz
app/
build/
install/
log/
EOF

# ---------- git init + commit ----------
say "提交到本地 git"
if [ "$WITH_DRIVER" = 1 ]; then DRIVER_TAG="（含驱动源码）"; else DRIVER_TAG="（不含驱动源码，部署时自动克隆）"; fi
# 提交署名：环境变量 → 你的全局 git 配置 → 兜底 jt128
AUTHOR_NAME="${GIT_AUTHOR_NAME:-$(git config --get user.name 2>/dev/null)}"
AUTHOR_EMAIL="${GIT_AUTHOR_EMAIL:-$(git config --get user.email 2>/dev/null)}"
AUTHOR_NAME="${AUTHOR_NAME:-jt128}"; AUTHOR_EMAIL="${AUTHOR_EMAIL:-jt128@localhost}"
info "提交署名：$AUTHOR_NAME <$AUTHOR_EMAIL>（可用 GIT_AUTHOR_NAME/GIT_AUTHOR_EMAIL 覆盖）"
(
  cd "$OUT_DIR" || exit 1
  git init -q -b "$BRANCH" 2>/dev/null || { git init -q && git checkout -q -b "$BRANCH"; }
  git add -A
  if git diff --cached --quiet; then
    echo "内容与上次一致，无需新提交"
  else
    git -c user.name="$AUTHOR_NAME" -c user.email="$AUTHOR_EMAIL" \
        commit -q -m "JT128 现场测试工具包：一键入口 + 裸包自检 + 内存监控 + 验收清单 + 移植部署${DRIVER_TAG}" \
      || die "git 提交失败"
  fi
) || die "git 提交失败"
FILES=$(cd "$OUT_DIR" && git ls-files | wc -l)
ok "仓库现有 $FILES 个文件"

# ---------- 推送 ----------
if [ -n "$REMOTE" ]; then
  say "推送到 $REMOTE"
  ( cd "$OUT_DIR" && git remote remove origin 2>/dev/null; git remote add origin "$REMOTE" ) || die "加远端失败"
  if ( cd "$OUT_DIR" && git push -u origin "$BRANCH" ); then
    ok "推送成功"
    # 远端默认分支不是我们推的这条时，小电脑 git clone 会检出空目录（最迷惑的故障之一）
    head_ref="$( cd "$OUT_DIR" && git ls-remote --symref origin HEAD 2>/dev/null | awk '/^ref:/{print $2}' )"
    if [ -n "$head_ref" ] && [ "$head_ref" != "refs/heads/$BRANCH" ]; then
      warn "远端默认分支是 ${head_ref#refs/heads/}，不是你刚推的 $BRANCH"
      info "小电脑上必须指定分支：git clone -b $BRANCH <仓库地址> jt128_test"
      info "或到 GitHub 仓库 Settings → Branches 把默认分支改成 $BRANCH（推荐，省得以后忘）"
    fi
  else
    warn "推送失败（多半是认证/权限）"
    info "① 用 PAT：git remote set-url origin https://<用户名>:<token>@github.com/<用户名>/<仓库>.git"
    info "② 或 SSH：先在 GitHub 加公钥（ssh-keygen -t ed25519，cat ~/.ssh/id_ed25519.pub）"
    info "③ 再执行：cd $OUT_DIR && git push -u origin $BRANCH"
  fi
else
  say "还没配远端，桌面机上执行下面两条即可"
  info "① 浏览器新建**私有**仓库（页面能用代理就够），例如 jt128-test-kit，不要勾选自动加 README"
  info "   https://github.com/new"
  info "② 回到终端："
  printf '\n     cd %s\n     git remote add origin https://github.com/<你的用户名>/jt128-test-kit.git\n     git push -u origin %s\n\n' "$OUT_DIR" "$BRANCH"
  warn "没装 gh CLI，只能走网页新建；若 push 卡在认证，用 PAT 当密码"
fi

say "小电脑上这样用"
info "git clone -b $BRANCH <仓库地址> jt128_test && cd jt128_test    # -b 指定分支，避免默认分支不一致"
info "./jt128/port/deps_check.sh                 # 依赖自检"
info "./jt128/port/port_setup.sh \"\$PWD\"          # 部署（驱动 + 编译 + 修 config）"
info "source ~/.bashrc && ./jt128/launch.sh radar"
info "本地仓库位置：$OUT_DIR（没推成功就整体 scp 过去，一样能用）"
