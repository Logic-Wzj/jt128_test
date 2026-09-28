# JT128 测试流程（禾赛 128 线机械式激光雷达）

环境：Ubuntu 22.04 + ROS 2 Humble（本机），现有导航仓库 `~/cod_-rm2026_-navigation`（原来用 Livox Mid-360）。
已有材料：`~/下载/JT128_用户手册_J01-zh-260720.pdf`、`/mnt/d_drive/linux_download/LidarUtilities_Sample_JT128_3.0.5_65677db_ubuntu22.out`。

## 名词澄清：LidarUtilities ≠ 驱动

手册 2.5「辅助工具」表里厂家自己就是分成三类的：

| 工具 | 用途 | 获取方式 |
|---|---|---|
| PandarView 2 | **点云可视化**、录/放包 | 官网下载页（公开版） |
| LidarUtilities | **设置参数、查看雷达信息、升级固件** | 联系禾赛技术支持 |
| SDK、ROS 驱动 | **辅助开发**（你的程序调用它） | https://github.com/HesaiTechnology |

所以你下的那个 `.out` 是**上位机（GUI 工具）**，不是驱动。把它拆开看了下内部结构（PyInstaller 归档，541 个条目 + 2216 个 Python 模块）：

- GUI 框架：`PySide6`（Qt6）+ `pyqtgraph` + `OpenGL` → 自带点云显示
- 多型号支持：`at128 / at320 / at360 / at512 / at720 / at_m21 / atx_jbox / et25 / ft120 / ot128 / jt16 / **jt128** / mt5 ...`
- JT128 专属模块：`jt128.cloud`（点云）、`jt128.view`、`jt128.info`、`jt128.ptc`、`jt128.ctrl`、`jt128.freeze`、`jt128.ui.ui_tab{parameterview,settingsview,factoryview}`、`jt128.sample.{view,info,ptc}`
- 通用界面：`lidar.ui.ui_mainwindow` + `ui_tab{home,parameter,settings,monitor,udp,ptc,register,upgrade,log,diagnosis,debug,integrationtest,uds}view`
- 协议栈：`lidar.udpnm`、`lidar.ptc`、`lidar.serial`、`lidar.xcp`、`lidar.uds`、`utils.parseFreezeFile`

即：**参数/信息/升级/点云查看/诊断**它都能干（所以第 4 步值得先跑它），但它是个窗口程序，**不会被你的 ROS 节点调用**。要在机器人上出 `/points` 话题，用的是 SDK/ROS 驱动。

## 0. 关键事实（来自用户手册 J01-zh-260720）

| 项目 | 值 |
|---|---|
| 通道 / 测距 | 128 线 / 0.5~60 m（40 m @10% 反射率），准度 ±3 cm、精度 2 cm |
| 视场 | 水平 360°，垂直 -4.4°~90.5° |
| 帧率 / 角分辨率 | 10 Hz（0.4°）、20 Hz（0.8°）；转速 600 / 1200 RPM |
| 回波模式 | 出厂默认 **双回波 最后及最强**（带宽 ×2） |
| 数据 | 以太网 100BASE-TX；单回波 41.26 Mbps，双回波 82.51 Mbps |
| 雷达 IP | 源 192.168.1.201/24，点云广播到 255.255.255.255:**2368**，故障信息 2369 |
| 主机 IP | 192.168.1.X（X=2~200、202~254），掩码 255.255.255.0 |
| 供电 | 9~32 V DC，**电源至少 2.6 A / 30 W**，典型功耗 8.4 W（10 Hz, 25℃）；低温/20 Hz 更高 |
| 点云包 | UDP 1100 B（以太网帧 1146 B）：包头 6 + 数据头 6 + 主体 1032 + 尾 56 |

## 1. 上电前

1. 摘下光罩外侧保护棉。
2. **断电插拔**连接器（热插拔可能击穿），M8 螺母扭矩 0.4 Nm。
3. 针脚：`6 红 = Power`，`1 黑 = GND`；以太网 `4/5 = TX±`、`3/2 = RX±`（雷达 TX 要接主机 RX）；`8 白 = GNSS PPS`、`7 绿 = NMEA`（基础测试可不接）。
4. 电源正负、电压范围先量一次；线缆压降不能让雷达端口处低于 9 V（手册附录 C）。
5. Class 1 人眼安全 905 nm，但别直视；可用手机摄像头看是否出光。

## 2. 主机网络（本机已就绪的脚本）

> ⚠️ **先检查雷达 IP 有没有被 Clash 劫持（不用无脑放行）**
> 本机 Clash 的 **TUN 目前是关的**（`tun.enable: false`，`enable_auto_launch: false`，只有后台 service 自启），
> 此刻没有劫持。**只有打开 TUN 模式时**，`192.168.1.201` 才会被 fake-ip（`198.18.0.0/15` + 策略路由 table 2022）劫持，
> 那时驱动会假装 `ptc connect success`（其实连到 Clash），UDP 点云收不到。
>
> 一键检查：`jt128route`（等价于 `ip route get 192.168.1.201`）
> - ✅ `dev enp131s0 ... src 192.168.1.100` 或 `via 10.132.255.254 dev enp131s0` → 没被劫持，不用管
> - ❌ `via 198.18.0.2 dev Meta table 2022` → 被劫持，需要放行
>
> 放行要写在**持久位置**：Clash Verge 的「全局扩展配置(Merge)」里加 `tun.route-exclude-address: [192.168.1.0/24]`，
> 或 GUI 的 TUN 设置里加。**直接改 `clash-verge.yaml` 没用**（Verge 每次生成配置会覆盖，目录里那些 `*.bak-regen-*` 就是证据）。
> 详见 `接上雷达后.md` 第①节。

```bash
cd ~/jt128
sudo ./setup_host.sh enp131s0 192.168.1.100        # 配 IP + ping 雷达
sudo ./setup_host.sh enp131s0 192.168.1.100 --tcpdump   # 顺带抓 5 秒包
```

等价于手册里的 `sudo ifconfig enp131s0 192.168.1.100`。
本机 `enp131s0`（Realtek RTL8125 2.5G）当前是校园网 `10.132.198.234/16`，**加第二个 IP 不冲突**；但正式测性能时建议雷达直连主机（手册也提示：网络里接交换机易丢包）。用完删除：`sudo ip addr del 192.168.1.100/24 dev enp131s0`。

## 3. 最裸验证：确认真的有数据（不需要任何厂商软件）

```bash
# 看有没有 1146 字节、源 192.168.1.201 的 UDP 包
sudo tcpdump -i enp131s0 -n -c 5 udp port 2368

# 按秒统计：包率 / 带宽 / 帧率 / 丢包 / 转速 / 温度 / 最近点
python3 ~/jt128/jt128_check.py --iface enp131s0 --seconds 20
```

期望值：

| 工况 | 包/s | 带宽 |
|---|---|---|
| 10 Hz 单回波 | ≈ 4500 | ≈ 41 Mbps |
| 10 Hz 双回波（默认） | ≈ 9000 | ≈ 82 Mbps |
| 20 Hz 双回波 | ≈ 18000 | ≈ 165 Mbps |

帧率应 ≈10/20 Hz，转速 ≈600/1200 RPM，丢包 0%。**这一条通过，说明供电、线缆、IP、出光、电机全部正常**，剩下的都是软件问题。

（`jt128_check.py --selftest` 可无雷达自检解析逻辑，已通过。）

## 4. 上位机 LidarUtilities（你下载的那个 .out —— 是工具，不是驱动）

它是 PyInstaller + PySide6 打包的 ELF（Python 3.10 / Qt6，内含 pyqtgraph + OpenGL），本机 `libxcb-cursor0` 等依赖齐全，能跑。先从 NTFS 盘拷到本地再执行：

```bash
mkdir -p ~/jt128/app
cp "/mnt/d_drive/linux_download/LidarUtilities_Sample_JT128_3.0.5_65677db_ubuntu22.out" ~/jt128/app/
chmod +x ~/jt128/app/*.out
cd ~/jt128/app && ./*.out
```

可看/改：源 IP、目的 IP、UDP 端口、转速（600/1200 RPM）、**回波模式**、Azimuth FOV、待机/运行、固件版本、雷达信息，Sample 版通常带点云显示。
建议第一步把回波模式改成**单回波**（第一/最强/最后任一）：带宽减半、点云减半，驱动好配，先用它把链路跑通。

配套：官网下载页的 **PandarView 2**（公开版）看/录点云、导入**角度修正文件**（每台雷达随附，点云排列异常多半是没导入）。

## 5. ROS 2 接入（最终目的）

现有 `livox_ros_driver2` 不认 JT128，要用禾赛官方 ROS2 驱动：

```bash
# GitHub 直连可用（2026-09 实测 HTTP 200 + git ls-remote 成功）；
# 若遇到抽风/超时，再开 Clash 代理（git config --global http.proxy http://127.0.0.1:7890）
cd ~/your_ws/src && git clone https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
# 建议固定到实测过的版本（当前上游 master 就是它）
cd HesaiLidar_ROS_2.0 && git checkout v2.0.12 && git submodule update --init --recursive
cd ~/your_ws && colcon build --symlink-install && source install/setup.bash
```

配置要点：
- `lidar_ip: 192.168.1.201`、`udp_port: 2368`；
- **回波模式必须和雷达当前设置一致**（双回波/单回波），否则点云数量和时间戳错乱——这是最常见的坑；
- 指定角度修正文件；`frame_id` 按你 URDF 命名；
- 启动后验证：`ros2 topic hz`、`ros2 topic echo --once`、`rviz2` 看 `PointCloud2`。单回波 10 Hz 每帧 115200 点，双回波 230400 点。

接你现有导航：
- `cpp_lidar_filter` 的 `input_topic` 默认 `/livox/lidar`，要 remap/改参数到 Hesai 话题，输出仍给 nav2 的 `/livox/lidar_filtered`（或改 nav2 参数里的 topic）；
- 双回波 2.3 M 点/s 对下游是压力（滤波节点、代价地图、LIO 都会有感），要么切单回波，要么用 JT128 自带的 **Azimuth FOV** 只输出前方扇区，再叠加 VoxelGrid 下采样；
- LIO（`small_point_lio`）从非重复扫描的 Livox 换到机械式 128 线，外参、时间戳、点云特征都要重新调。

## 6. 建议的测试项（比赛视角）

1. **链路**：连续跑 10~30 min，`jt128_check.py` 盯丢包/帧率/温度；0 丢包、帧率稳定。
2. **测距准度/精度**：白墙或反光板，卷尺量 1/3/5/10 m，比对点云距离（±3 cm 准度、2 cm 精度）。
3. **视场覆盖**：垂直 -4.4°~90.5°、水平 360°，看近处/地面/盲区，特别是有没有裁掉需要的扇区（可用 Azimuth FOV 裁剪验证省带宽效果）。
4. **量程**：60 m 极限、40 m@10% 反射率，看远距离点密度与噪声。
5. **回波模式对比**：单回波 vs 双回波，密度/带宽/下游算力取舍。
6. **时间同步**（如需多传感器融合）：GNSS PPS+NMEA 或 PTP；无外部源时是内部时钟，多雷达/相机融合会飘。
7. **与 Livox 对比**：同为 LIO 输入，机械式 128 线 vs 非重复扫描的建图/定位效果。

## 7. 故障排查速查（手册第 6 章）

| 现象 | 先查 |
|---|---|
| 电机不转 | 电源接触、电压/电流 ≥2.6 A、是否处于待机模式 |
| 转但没数据 | 网线、Destination IP、固件版本、是否真的出光（红外） |
| tcpdump 有包、软件没数据 | 端口 2368、VLAN ID、主机防火墙、软件版本 |
| 上位机连不上 | 主机 IP 是否同网段（192.168.1.x）、VLAN、换机器/重启 |
| 丢包 | 网络过载、**别接交换机**、只接一台雷达、重新上电 |
| 点云排列异常/闪烁/视场残缺 | 光罩脏污、角度修正文件、转速是否平稳、内部温度 -20~95℃、丢包 |
| GNSS 锁不上 | GNSS 接线、PPS 输入、GNSS Destination Port、电气规格 (3.3 V, 周期 1 s) |

## 8. 方案 A 快速上手（zsh）

驱动在独立工作区 `~/hesai_ws` 编译，**cod 仓库一行不改**；点云经 `cpp_lidar_filter` 后仍发到 nav2 认的 `/livox/lidar_filtered`。

```bash
# 一次性：把 zsh 片段挂上（或手动 source）
echo 'source ~/jt128/jt128.zsh' >> ~/.zshrc

jt128env        # 加载 humble + hesai_ws + cod_ws
jt128up         # 起「驱动 + 滤波 + base_link->front_jt128 静态TF」
jt128verify     # 一键验证：进程/话题/频率/点数/frame_id/丢包/TF/资源
jt128mem        # 盯内存/CPU/FD/内核缓冲
jt128check      # 裸 UDP 收包统计（不依赖 ROS）
jt128down       # 清理所有 JT128 进程（别用 timeout ros2 launch，会留孤儿进程）
```

zsh 片段里的命令都在 `~/jt128/jt128.zsh`，也可以不挂 `.zshrc`，用时 `source ~/jt128/jt128.zsh` 即可。

手动等价命令（zsh）：

```zsh
source ~/hesai_ws/install/setup.zsh
source ~/cod_-rm2026_-navigation/install/setup.zsh
ros2 launch ~/jt128/launch/jt128_nav_test.py
```

## 9. 搬到小电脑（不用 U 盘）

桌面机整理 → 推到自己的 GitHub 仓库 → 小电脑 clone 下来部署：

```bash
# 桌面机
bash ~/jt128/port/push_to_github.sh --dir ~/jt128_repo --with-driver
cd ~/jt128_repo && git remote add origin git@github.com:Logic-Wzj/jt128_test.git && git push -u origin main

# 小电脑
git clone git@github.com:Logic-Wzj/jt128_test.git jt128_test && cd jt128_test
./jt128/port/deps_check.sh          # 依赖自检
./jt128/port/port_setup.sh "$PWD"   # 部署：驱动 + 编译 + shell 入口 + 修正 config.yaml
source ~/.bashrc && ./jt128/launch.sh radar
```

- `push_to_github.sh` 会自动排除上位机 97 MB 二进制、`app/`、`__pycache__` 等，并在存在 >10 MB 文件时报错退出；
- 不加 `--with-driver` 就是轻量版（360 KB），小电脑部署时自动从禾赛官方克隆驱动（带重试）；
- 完整说明（含连不上 GitHub 时用 scp、私有/公开选择、后续更新流程）见 `GitHub传包.md`；
- 也可以继续用压缩包：`bash ~/jt128/port/make_bundle.sh ~/`（产物 `~/jt128_port_<日期>.tar.gz`）。

