# 真雷达接仿真（HIL）启动方式

前提：雷达网线接 `enp131s0`，已配 `192.168.1.100/24`（`sudo ~/jt128/launch.sh net enp131s0 192.168.1.100`），
雷达上电后 `cat /sys/class/net/enp131s0/statistics/rx_packets` 每秒涨 ~9000。

**开三个终端，按顺序来。**

## 终端 1 — 雷达驱动

```bash
cd ~/jt128
./launch.sh radar          # 查网卡→查 Clash 劫持→2 秒探包→起驱动（含 64MB DDS profile）
```

正常输出：`/lidar_points` **10 Hz**、230400 点/帧（双回波）、丢包 0。
只想起驱动、跳过检查就用 `./launch.sh driver`。

## 终端 2 — 仿真 + 专用 RViz

```bash
source /opt/ros/humble/setup.bash
source ~/cod_-rm2026_-navigation/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/jt128/sim/fastdds_large_msg.xml

ros2 launch cod_sim fzsd_sim_launch.py \
    params_file:=$HOME/jt128/sim/gt_sim_jt128_params.yaml \
    use_rviz:=true
```

- `params_file:=` 指向**我改过的副本**：局部代价地图的障碍层从 `/red_standard_robot1/rplidar_a2/scan`
  换成 **`/jt128/points`（PointCloud2）**，并把地图路径从不存在的 `~/fzsd2025/...` 修正到 cod 安装目录。
  cod 仓库本身**没被改动**。
- `use_rviz:=true` 会拉起 cod 的专用 RViz（配置 `cod_bringup/rviz/cod_nav.rviz`）。
- **一定要在自己的终端里跑**（后台/无 display 环境下 `ign gazebo` 会 `could not connect to display` 段错误）。

等它就绪：Gazebo 里机器人出现 + 下面这条能出数字：

```bash
ros2 run tf2_ros tf2_echo odom front_mid360      # 应打印 Translation: [-4.955, 3.191, 0.494]
```

## 终端 3 — HIL 中继（把真雷达点云灌进仿真代价地图）

```bash
cd ~/jt128
./launch.sh hil source:=real start_driver:=false bridge_gimbal_tf:=false
```

两个参数都不能省：

- `start_driver:=false`：驱动已在终端 1 跑，再起一个会抢 2368 端口。
- `bridge_gimbal_tf:=false`：**2026-09-28 实测，单个干净仿真里 TF 树是完整的**
  （`odom→front_mid360` 直接通），旧文档里说"缺 `chassis→gimbal_yaw`、需要补静态 TF"是当时双仿真
  场景的假象。若开了这个补丁，静态 TF 会和 `gimbal_yaw` 关节的实时 TF 打架。

## 验证

```bash
# 中继输出：10 Hz / frame_id front_mid360 / 230400 点
ros2 topic hz /jt128/points
ros2 topic echo --once --field header.frame_id /jt128/points
ros2 topic echo --once --field width /jt128/points

# 代价地图在标记
ros2 topic hz /local_costmap/costmap_updates
python3 ~/jt128/costmap_occ.py            # 局部代价地图占用格数
```

## ⚠️ 已知坑：`controller_server` 停在 unconfigured → 代价地图完全不工作

今天的现象：中继输出一切正常（10 Hz、帧对、点数对），但 `/local_costmap/costmap_updates` **没有更新**、
`/local_costmap/costmap` **根本收不到**。根因是生命周期没激活：

```bash
ros2 lifecycle get /controller_server     # → unconfigured [1]
```

`local_costmap` 是 `controller_server` 的子节点，所以它不激活就什么都不发、障碍层也不标记。

最可能是**启动竞态**：nav2 在 Gazebo 还没生成机器人、`gimbal_yaw_fake` 这个 TF 还不存在时就去
configure 代价地图，超时后永久停在 unconfigured。仿真日志里对应的报错是：

```
[local_costmap.local_costmap]: Timed out waiting for transform from gimbal_yaw_fake to odom
to become available, tf error: Invalid frame ID "gimbal_yaw_fake" ... frame does not exist
```

处理顺序：

1. **先起仿真，等机器人真的出现**（`tf2_echo odom front_mid360` 出数）**再起中继**；中继晚一点无所谓。
2. 若 nav2 仍停在 unconfigured，手动拉起来（`local_costmap` 会跟着激活）：
   ```bash
   ros2 lifecycle set /controller_server configure
   ros2 lifecycle set /controller_server activate
   ros2 lifecycle get  /controller_server      # 应为 active [3]
   ```
3. 还是不行就把 `~/.ros/log/` 里最新的 `controller_server` 报错发出来。

## 数据流（便于排查）

```
真雷达 ──UDP 9000 包/s──> hesai_ros_driver ──/lidar_points (front_jt128, 10Hz, 5.99MB/帧)──>
   jt128_sim_relay ── 改 frame: front_jt128→front_mid360，时间戳校到仿真时钟 ──>
   /jt128/points ──> 仿真 local_costmap 的 obstacle_layer（obstacle_min_range 0.3，
                     高度过滤 0.10~1.50 m）
```

一次调不通时按这个顺序看：`/lidar_points`（终端 1）→ `/jt128/points`（终端 3）→
`/local_costmap/costmap` 是否存在（生命周期）→ 占用格数是否变化（标记）。
