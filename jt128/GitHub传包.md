# 用 GitHub 把工具包搬到小电脑（不用 U 盘）

场景：桌面机（这台，x86 + 图形界面）整理好后推到**你自己的 GitHub 仓库**，小电脑（机器人主机，i7 + 图形界面 + 已有 cod 工作区）`git clone` 下来直接用。

## 一、推荐做法：仓库自带驱动源码（一次 clone 到位）

**为什么带驱动**：小电脑只跟你的仓库打交道，不再需要访问禾赛官方仓库。轻量做法虽然更小，但小电脑部署时还要额外去 GitHub 拉禾赛驱动和它的 SDK 子模块（这一步网络抖动会失败，实测遇到过一次），少一个外部依赖就少一个现场故障点。2.7 MB 的代价换来的确定性很值。

### 桌面机（3 步）

```bash
# 1) 整理成一个干净的本地仓库（自动排除上位机二进制、__pycache__、app/ 等）
bash ~/jt128/port/push_to_github.sh --dir ~/jt128_repo --with-driver

# 2) 浏览器新建仓库（https://github.com/new，不要勾 "Add a README file"）
#    本机用的仓库：https://github.com/Logic-Wzj/jt128_test （已建好、首次已推送）

# 3) 推上去
cd ~/jt128_repo
git remote add origin git@github.com:Logic-Wzj/jt128_test.git
# ↑ 如果报 "远程 origin 已经存在"，就不用再加，改用：
#   git remote set-url origin git@github.com:Logic-Wzj/jt128_test.git
git remote -v          # 确认地址是 git@github.com:... 开头
git push -u origin main
```

**认证不用 PAT**：本机 `~/.ssh/id_ed25519` 已经加到 GitHub 了（`ssh -T git@github.com` 会回 `Hi Logic-Wzj!`），所以用 `git@github.com:` 的 SSH 地址免密。只有走 HTTPS 地址（`https://...`）时才需要 PAT（Settings → Developer settings → Personal access tokens → 勾 `repo` → 生成后当密码用，只显示一次）。

> 本机到 GitHub 的连通性时不时抽风（校园网直连会被掐、Clash 时好时坏）：`curl https://github.com` 卡住或 TLS 被重置就换个时间/网络重试，SSH 通道相对稳。`git push` 卡住先 Ctrl-C，别反复重试到超时。

### 小电脑（4 步）

```bash
git clone -b main git@github.com:Logic-Wzj/jt128_test.git jt128_test   # 目录叫 jt128_test 没问题
cd jt128_test
./jt128/port/deps_check.sh            # 1. 依赖自检（应无 ❌）
./jt128/port/port_setup.sh "$PWD"     # 2. 部署：装驱动 + 编译 + 写 shell 入口
source ~/.bashrc                      # 3. 让 jt128 入口生效（zsh 用 source ~/.zshrc）
./jt128/launch.sh radar               # 4. 配网 + 探包 + 启动驱动（一键）
```

> `-b main` 是保险：如果你的仓库默认分支是 `master`（或别的），不带 `-b` 会 clone 出一个**空目录**（命令不报错，最迷惑）。脚本 push 完会自己检查远端默认分支，不一致时直接提示你。
> 一劳永逸的办法：GitHub 仓库 → Settings → Branches → 把默认分支改成 `main`。

`port_setup.sh` 做的事：把 `jt128/` 装到 `~/jt128`、驱动源码放到 `~/HesaiLidar_ROS_2.0`、在 `~/hesai_ws` 里编译驱动 + 中继 + Livox 兼容层、修正 `config.yaml` 里的绝对路径和 JT128 专用项、写入 `~/.bashrc`/`~/.zshrc` 的 `jt128` 入口。**换机必做**，因为它把路径全部改成本机路径。

## 二、轻量做法（仓库只放脚本，360 KB）

```bash
bash ~/jt128/port/push_to_github.sh --dir ~/jt128_repo     # 不加 --with-driver
cd ~/jt128_repo && git remote add origin git@github.com:Logic-Wzj/jt128_test.git && git push -u origin main
```

小电脑步骤完全一样，只是 `port_setup.sh` 发现包里没有驱动源码时，会自己从禾赛官方克隆（固定 tag `v2.0.12`，BSD-3，LICENSE 随克隆保留），带 3 次重试；如果上次克隆中断留下缺 SDK 的坏目录，重跑会自动补齐。

需要换源（镜像/内网 git）时：

```bash
HESAI_DRIVER_URL=<镜像地址> ./jt128/port/port_setup.sh "$PWD"
HESAI_DRIVER_REF=          # 置空则跟随上游 HEAD，不固定 tag
```

## 三、小电脑连不上 GitHub 怎么办

GitHub 走不通时，直接把整理好的目录整体拷过去（局域网 scp / 移动硬盘都行），后续步骤一模一样：

```bash
# 桌面机：目录已经在 ~/jt128_repo（上面第 1 步产物），直接拷
scp -r ~/jt128_repo <用户名>@<小电脑IP>:~/
# 或者仍然用原来的压缩包
scp ~/jt128_port_*.tar.gz <用户名>@<小电脑IP>:~/

# 小电脑：解开后进目录，照样跑 port_setup.sh
cd ~/jt128_repo && ./jt128/port/deps_check.sh && ./jt128/port/port_setup.sh "$PWD"
```

## 四、什么**不要**往仓库里放

| 东西 | 原因 | 怎么办 |
|---|---|---|
| `LidarUtilities_Sample_JT128_*.out`（97 MB 上位机 GUI） | 禾赛专有二进制，不能公开再分发；且超过 GitHub 适用体积 | 留在桌面机；真要用 scp/U 盘单独拷，或放私有仓库的 Release 附件 |
| `app/`、`__pycache__/`、`*.npy`、`build/`、`install/`、`log/` | 本机产物，没用还占地方 | 脚本已自动排除，`.gitignore` 也写了 |
| `~/.zshrc`、网络配置、日志 | 含本机信息 | 不用管，部署脚本会生成 |

脚本里有硬检查：整理出来的目录只要存在 **>10 MB 的文件**就直接报错退出并列出文件，防止误传大文件。含驱动源码时请**保留所有 LICENSE 文件**（Modified BSD-3 要求保留版权声明；脚本原样复制驱动目录，不会删 LICENSE）。

## 五、以后改了脚本怎么更新

```bash
bash ~/jt128/port/push_to_github.sh --dir ~/jt128_repo --with-driver   # 重跑，会自动保留 .git
cd ~/jt128_repo && git push                                            # 直接推，不用 --force
```

小电脑上：

```bash
cd ~/jt128_test && git pull && ./jt128/port/port_setup.sh "$PWD"
```

> 注意：`port_setup.sh` 对已存在的 `~/HesaiLidar_ROS_2.0` 默认**跳过**（不覆盖你在机器上改过的驱动配置），所以更新脚本本身会生效，驱动的改动不会被冲掉。

## 六、常见问题

- **clone 出来是空目录**：远端默认分支不是 `main`。用 `git clone -b main <地址>`，或去仓库 Settings → Branches 改默认分支。
- **`git push` 卡住/超时**：先确认浏览器能开 github.com；用 PAT 时注意不要带空格。国内网络可试 SSH（22 端口）或 `https://` 加代理：`git config --global http.proxy http://127.0.0.1:7890`。
- **小电脑 `deps_check.sh` 报 ❌**：照着它给的安装命令补包，然后重跑。
- **小电脑仓库名/目录名是 `jt128_test` 有影响吗**：没有。所有脚本都按"自己所在位置"定位（`BASH_SOURCE`），`port_setup.sh` 只认你传给它的那个目录参数。
- **不想让别人看到**：新建仓库时选 Private，私有仓库同样能 `git clone`（需要认证）。
