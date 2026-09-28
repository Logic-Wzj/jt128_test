# 真雷达接仿真（HIL）启动方式

> **你的终端是 zsh** → 用下面「方式 A」，一行一条，不需要手动 source 任何东西
> （`~/.zshrc` 里已经 source 了 `/opt/ros/humble/setup.zsh` 和 `~/jt128/jt128.zsh`）。
> **不要** source colcon 的 `setup.bash`：zsh 里它的前缀推导会出错，报
> `没有那个文件或目录: .../setup.sh`、`Package 'cod_sim' not found`。要手动 source 就用 `.zsh` 版本。

前提：雷达网线接 `enp131s0`，已配 `192.168.1.100/24`（`sudo ~/jt128/launch.sh net enp131s0 192.168.1.100`），
雷达上电后 `cat /sys/class/net/enp131s0/statistics/rx_packets` 每秒涨 ~9000。

---

## 方式 A：zsh（推荐，你的环境）

**终端 1 — 雷达驱动**
```zsh
~/jt128/launch.sh radar
```
（这是个 bash 脚本，从 zsh 调用没问题。正常输出 `/lidar_points` 10 Hz、230400 点/帧、丢包 0。）

**终端 2 — 仿真 + 专用 RViz**
```zsh
jt128sim
```
`jt128sim` 内部已经做了：检查有没有仿真在跑（防止起两个）→ `jt128env`（加载 humble + hesai_ws + cod_ws）
→ 注入 64 MB DDS profile → `ros2 launch cod_sim fzsd_sim_launch.py params_file:=<jt128 版参数>`。
`use_rviz` 默认就是 True，所以**专用 RViz（`cod_bringup/rviz/cod_nav.rviz`）会自动弹出**；不想要就 `jt128sim use_rviz:=false`。

等它就绪：
```zsh
ros2 run tf2_ros tf2_echo odom front_mid360     # 应打印 Translation: [-4.955, 3.191, 0.494]
```

**终端 3 — HIL 中继**（仿真 TF 出来之后再起）
```zsh
jt128hil source:=real start_driver:=false bridge_gimbal_tf:=false
```

两个参数都别省：

- `start_driver:=false`：驱动已在终端 1 跑，再起一个会抢 2368 端口。
- `bridge_gimbal_tf:=false`：**2026-09-28 实测，单个干净仿真里 TF 树是完整的**（`odom→front_mid360` 直接通）。
  旧文档里"缺 `chassis→gimbal_yaw`、要补静态 TF"是当时双仿真场景的假象；补了会和 `gimbal_yaw` 关节的实时 TF 打架。

**验证**
```zsh
ros2 topic hz /jt128/points                                    # 10 Hz
ros2 topic echo --once --field header.frame_id /jt128/points   # front_mid360
ros2 topic hz /local_costmap/costmap_updates                   # 有更新 = 障碍层在标记
python3 ~/jt128/costmap_occ.py                                 # 占用格数
jt128status                                                    # 总览
```
收工：`jt128down`（只清 JT128 相关），仿真/RViz 在各自终端 Ctrl-C。

---

## 方式 B：bash 终端（显式 source）

```bash
source /opt/ros/humble/setup.bash
source ~/cod_-rm2026_-navigation/install/setup.bash
source ~/hesai_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/jt128/sim/fastdds_large_msg.xml

ros2 launch cod_sim fzsd_sim_launch.py \
    params_file:=$HOME/jt128/sim/gt_sim_jt128_params.yaml \
    use_rviz:=true
```
zsh 里等价的显式写法（`.zsh` 后缀）：
```zsh
source /opt/ros/humble/setup.zsh
source ~/cod_-rm2026_-navigation/install/setup.zsh
source ~/hesai_ws/install/setup.zsh
```

- `params_file:=` 指向**我改过的副本**：局部代价地图障碍层从 `/red_standard_robot1/rplidar_a2/scan`
  换成 **`/jt128/points`（PointCloud2）**，地图路径从不存在的 `~/fzsd2025/...` 修正到 cod 安装目录。
  **cod 仓库本身没有被改动。**
- **GUI 一定要在你自己终端里跑**：无 display 的后台环境里 `ign gazebo` 会
  `qt.qpa.xcb: could not connect to display` 直接段错误。

---

## ⚠️ 已知坑：`controller_server` 停在 unconfigured → 代价地图完全不工作

2026-09-28 现象：中继输出一切正常（10 Hz、帧对、点数对），但 `/local_costmap/costmap_updates`
**没有更新**、`/local_costmap/costmap` **根本收不到**。根因是生命周期没激活：

```zsh
ros2 lifecycle get /controller_server     # → unconfigured [1]
```

`local_costmap` 是 `controller_server` 的子节点，它不激活就什么都不发、障碍层也不标记。

最可能是**启动竞态**：nav2 在 Gazebo 还没生成机器人、`gimbal_yaw_fake` 这个 TF 还不存在时就去
configure 代价地图，超时后永久停在 unconfigured。仿真日志里对应报错：

```
[local_costmap.local_costmap]: Timed out waiting for transform from gimbal_yaw_fake to odom
to become available, tf error: Invalid frame ID "gimbal_yaw_fake" ... frame does not exist
```

处理顺序：

1. **先起仿真，等机器人真的出现**（`tf2_echo odom front_mid360` 出数）**再起中继**。
2. 若 nav2 仍停在 unconfigured，手动拉起来（`local_costmap` 会跟着激活）：
   ```zsh
   ros2 lifecycle set /controller_server configure
   ros2 lifecycle set /controller_server activate
   ros2 lifecycle get  /controller_server      # 应为 active [3]
   ```
3. 还是不行，把 `~/.ros/log/` 里最新的 `controller_server` 报错发出来。

---

## 数据流（便于排查）

```
真雷达 ──UDP 9000 包/s──> hesai_ros_driver ──/lidar_points (front_jt128, 10Hz, 5.99MB/帧)──>
   jt128_sim_relay ── 改 frame: front_jt128→front_mid360，时间戳校到仿真时钟 ──>
   /jt128/points ──> 仿真 local_costmap 的 obstacle_layer（obstacle_min_range 0.3，
                     高度过滤 0.10~1.50 m）
```

调不通时按这个顺序看：`/lidar_points`（终端 1）→ `/jt128/points`（终端 3）→
`/local_costmap/costmap` 是否存在（生命周期）→ 占用格数是否变化（标记）。

## 两种启动方式对照

| | 手动 ros2 launch | 快捷函数 |
|---|---|---|
| 环境 | 要自己 source 三个 setup（zsh 记得用 `.zsh`） | `jt128env` 内部搞定 |
| DDS profile | 要自己 export（漏了就只有 3~4 Hz） | 自动注入 |
| 重复起仿真 | 会起第二个，互相搞死 | 检测到就直接拒绝 |
| 专用 RViz | `use_rviz:=true` | 默认就是 True |
| 点云话题/参数 | 要记 `params_file:=` | 已写死成 jt128 版参数 |
