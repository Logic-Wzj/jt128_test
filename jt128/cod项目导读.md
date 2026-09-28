# cod_-rm2026_-navigation 项目通读笔记

> 阅读人：DSH（含 4 路并行深挖子调查）。所有结论后面都标了 `文件:行号`，便于你复核。
> 自研代码约 **2.09 万行 / 15 个包**；`src/fzsd_vendor` 另有 **3.8 万行 / 192 文件**（内嵌第三方，来自 fzsd2025 那套）。

## 1. 包清单与职责

| 包 | 行数 | 职责（一句话） |
|---|---|---|
| `cod_sim` | 9.3k | 仿真相关：Gazebo 世界/机器人 spawn、gt_odom、fake_odom、tf_forward、urdf_global、pcd_publisher、priest_bridge，以及 6 个 launch（仿真/loopback/priest/决策一键） |
| `small_point_lio` | 5.3k | 激光惯性里程计（LIO），真机定位来源；配置 `mid360.yaml`（Livox）与 `unilidar_l2.yaml` |
| `cod_bringup` | 2.8k | 真机 bringup：`singlenav_launch`（单机）/`multiplenav_launch`（多机）/`navigation_launch`（nav2 服务端）/`localization_launch`（仅 map_server）/`auto_save_map`；3 套 nav2 参数 + 地图 |
| `waypoint_editor` | 1.5k | 航点编辑/下发（3 个 launch） |
| `pb_omni_pid_pursuit_controller` | 1.4k | 全向底盘 PID 追击控制器（nav2 插件） |
| `cod_decision_bt` | 1.3k | 决策行为树：XML 树 + 自定义 BT 节点（BehaviTree.CPP v3），仿真树与真机树各一份 |
| `pb_nav2_plugins` | 0.9k | 自定义 nav2 插件集合 |
| `pointcloud_to_laserscan` | 0.9k | 点云→激光扫描（供 costmap/规划用） |
| `ros2_simple_serial` | 0.4k | **包名其实叫 `cod_serial_ul26`**（目录名≠包名！），串口底盘通信 |
| `cod_referee_simulator` | 0.3k | 裁判系统模拟器（供决策树在仿真里用） |
| `goal_approach_controller` | 0.2k | 包装 MPPI 的"接近目标"控制器 |
| `fake_vel_transform` | 0.2k | 造虚拟基座帧 `*_fake`，把 nav2 的输出坐标系与底盘解耦 |
| `cpp_lidar_filter` | 0.2k | 点云 CropBox 去车身（**降采样被注释掉了**，见 §6） |
| `cod_referee_interfaces` | 18 行 | 裁判消息/服务定义 |

## 2. 三条运行路径（入口与数据流）

### A. Gazebo 仿真（+裁判+决策）
```
ros2 launch cod_sim rm_decision_launch.py          # 一键：仿真 + 裁判 + 决策树（cod_sim/launch/rm_decision_launch.py:8-24）
├── cod_sim/fzsd_sim_launch.py                     # 仿真主体
│   ├── fzsd_gazebo_launch.py                      # 世界 + spawn 红蓝两台车 + 底盘控制 + 话题桥
│   ├── gt_odom  (Gazebo 真值 → odometry + TF odom→base_footprint)
│   ├── static TF map→odom  (x=5,y=-3,z=0.28)      # fzsd_sim_launch.py:94-103
│   ├── map_server + urdf_global + tf_forward
│   ├── cod_bringup/navigation_launch.py  ← params=gt_sim_params.yaml
│   └── rviz2
├── cod_referee_simulator/referee.launch.py
└── cod_decision_bt/decision_tree.launch.py        # 只起 decision_tree_node
```
- 世界/出生位姿来自 `rmu_gazebo_simulator/config/gz_world.yaml`（`fzsd_gazebo_launch.py:37-41`）。
- spawn 用 `-x=值` 等号格式规避 gflags 负数 bug —— **这是 cod_sim 复制一份 launch 的唯一原因**（`fzsd_gazebo_launch.py:3-6`）。
- 机器人生成：SDF xmacro → `sdformat_tools.UrdfGenerator` 转 URDF 给 RSP（`fzsd_gazebo_launch.py:31-32,67-69`）。
- 红车 RSP **不加命名空间重映射**（TF 直发全局），蓝车保留命名空间避免帧冲突（`fzsd_gazebo_launch.py:98-106`）。

### B. 无硬件 loopback（**没有 Gazebo**，最适合台架/HIL）
```
ros2 launch cod_sim loopback_sim_launch.py
├── static TF map→odom (yaw=-0.5)                  # loopback_sim_launch.py:43-52
├── fake_odom：积分 aft_cmd_vel → odom + TF odom→base_link   # :53-59
├── fake_vel_transform：base_link → base_link_fake           # :60-66
├── cod_bringup/navigation_launch.py + localization_launch.py ← **singlenav2_params.yaml（真机参数）** + maps/rmul2026.yaml
└── rviz2
```
> 注释明说：跳过 realsense / cod_serial / small_point_lio / cpp_lidar_filter（`loopback_sim_launch.py:1-11`），定位用固定 TF（无 AMCL）。

### C. 真机
```
ros2 launch cod_bringup singlenav_launch.py
├── cpp_lidar_filter: /livox/lidar → /livox/lidar_filtered   # singlenav_launch.py:36-50
├── small_point_lio (config/mid360.yaml)                     # :51-65
├── static TF map→odom (z=0.05, yaw=-0.5)                    # :67-89
├── fake_vel_transform                                        # :90-95
├── cod_serial_ul26/cod_serial（串口底盘）                     # :96-101
├── realsense2_camera（深度点云）                              # :102-119
├── navigation_launch + localization_launch ← singlenav2_params.yaml  # :120-139
└── rviz2
```
⚠️ **这个 launch 里没有雷达驱动**：Livox 驱动在 vendored 的 `fzsd_vendor/pb2025_nav_bringup/launch/rm_sentry_reality_launch.py:158-161` 里起。也就是说真机要么跑那个 fzsd launch，要么单独起驱动。

## 3. 坐标系与 TF 设计（最容易被坑的地方）

| 路径 | 主链 | nav2 robot_base_frame | local costmap global_frame |
|---|---|---|---|
| 真机 | map →(静态 yaw -0.5) odom → base_link(LIO) → **base_link_fake** | `base_link_fake`（singlenav2_params.yaml:5,184） | `odom`（:183） |
| loopback | 同上，base_link 由 fake_odom 积分 | 同上 | 同上 |
| 仿真 | map →(静态 x5,y-3) odom → base_footprint(gt_odom) → chassis → gimbal_yaw → front_mid360 | `gimbal_yaw_fake` | `odom` |

**关键机制（我踩过的坑）**：
- RSP 把**固定关节**发 `/tf_static`（如 `gimbal_yaw→front_mid360`），把**可动关节**发 `/tf`，而后者依赖 `/joint_states` 持续输入（Gazebo 桥提供）。
  → **Gazebo 一死，`chassis→gimbal_yaw` 就消失**，全局 TF 看起来"断成两棵树"，costmap 随即报 `Timed out waiting for transform ... to odom`。这不是配置错误，是时钟源/joint_states 断供的表现。
- `tf_forward.py` 把 `/red_standard_robot1/tf(_static)` 转发到全局（`:7-32`）；红车其实已经直发全局，它主要服务蓝车与 rviz 显示。
- `fake_vel_transform` 造 `base_link_fake`/`gimbal_yaw_fake`，让 nav2 的 `robot_base_frame` 与真实底盘帧解耦（真机 `singlenav_launch.py:90-95`，仿真 `loopback_sim_launch.py:60-66`）。

## 4. 感知与障碍源（真机 vs 仿真完全不同）

| | 真机 / loopback（singlenav2_params.yaml） | Gazebo 仿真（gt_sim_params.yaml） |
|---|---|---|
| costmap 插件 | `static_layer, **stvl_voxel_layer**, inflation_layer`（:192,281） | `static_layer, **obstacle_layer**, inflation_layer` |
| 观测源 | `livox_source`(**/livox/lidar_filtered**, PointCloud2) + `realsense_source`(/camera/camera/depth/color/points)（:218-244） | `scan`(**/red_standard_robot1/rplidar_a2/scan**, LaserScan)（gt_sim_params.yaml:201-210） |
| 定位 | small_point_lio（真机）/ fake_odom（loopback）/ 固定 map→odom | gt_odom（Gazebo 真值） |
| LIO 帧 | `livox_frame`（small_point_lio/config/mid360.yaml:6） | 无 LIO |

> 注意：真机 nav2 参数里写的 `lidar_frame: front_mid360`（reality_mppi_nav2_params.yaml:99 等）**该文件无人引用**，是死配置；真正生效的是 `singlenav2_params.yaml`。
> 本机已装 `spatio_temporal_voxel_layer` ✓，所以真机参数在本机能跑起来（对台架 HIL 是好消息）。

## 5. 参数文件地图

| 文件 | 被谁用 |
|---|---|
| `cod_bringup/params/singlenav2_params.yaml` | `singlenav_launch` / `loopback_sim_launch` / `navigation_launch`(默认) / `localization_launch`(默认) —— **真机默认** |
| `cod_bringup/params/multiplenav2_params.yaml` | `multiplenav_launch` |
| `cod_sim/config/gt_sim_params.yaml` | `fzsd_sim_launch.py:45-48` —— **仿真默认** |
| `cod_sim/config/fzsd_sim_params.yaml` + `priest_planner_params.yaml` | `priest_launch.py`（PRIEST 局部规划那条线） |
| `cod_bringup/params/reality_mppi_nav2_params.yaml` | **无人引用（死配置）** |
| `cod_bringup/maps/rmul2026.yaml` | 真机/loopback 地图（`localization_launch.py:76-79` 默认值） |

- `localization_launch.py` 用 `RewrittenYaml` 把 `yaml_filename` 覆盖为 launch 参数（`:56-58`, `:76-79`），所以 `singlenav2_params.yaml:360` 里那串 `/home/cod-sentry/dyx_ws/...` 的硬编码**不会**生效（本机确实没有该路径，但地图照常加载）。
- 定位节点只有 `map_server`（`localization_launch.py:44`），**没有 AMCL**；定位 = 固定 map→odom + LIO 的 odom→base_link。

## 6. 可疑点 / 坑（按要紧程度）

1. **`cpp_lidar_filter` 的降采样是注释掉的**（`cpp_lidar_filter/src/filter_node.cpp:106-113`），`leaf_size` 参数名存实亡；115200 点/帧量级的雷达（JT128）会原样灌给下游。
2. **真机 launch 不含雷达驱动**（见 §2C），依赖 vendored fzsd 的 reality launch 或单独启动 —— 换雷达型号时最容易被忽略的一环。
3. **仿真与真机的 costmap 插件不同**（obstacle_layer vs stvl_voxel_layer），把仿真参数直接搬到真机会起不来。
4. **`reality_mppi_nav2_params.yaml` 是死配置**，容易误改（我就差点按它分析）。
5. **`cod_serial_ul26` 包名≠目录名（`ros2_simple_serial`）**，按目录名找依赖会找不到。
6. **红/蓝车 TF 策略不对称**（红直发全局、蓝在命名空间，`fzsd_gazebo_launch.py:98-106`）—— 蓝车的帧带命名空间，跨车引用要小心。
7. **`tf_forward` 的动态 TF 用 volatile QoS，静态用 transient_local**（`tf_forward.py:24-27`）—— 后启动的监听者能拿到静态帧，但**动态关节帧依赖 joint_states 持续输入**（见 §3），Gazebo/桥一断就断链。
8. **两个仿真同时跑会互相挤死**（同机 gz-transport 端口冲突，先起的 Gazebo 消失、`/clock` 只剩订阅者）—— 我本人踩过，已加 `jt128ps`/启动守卫防呆。
9. **`/home/cod-sentry/...` 之类的跨机绝对路径**在配置里存在（虽然被 RewrittenYaml 救了），移植时值得全局搜一遍。

## 7. 与 JT128 HIL 的衔接（结论）

- **最佳 HIL 宿主不是 Gazebo，而是 `loopback_sim_launch.py`**：无 Gazebo（没有 clock/端口/GPU 问题）、`use_sim_time=false`（真实雷达的绝对时间戳不需要回退）、且**直接用真机参数**（`/livox/lidar_filtered` + stvl_voxel_layer）——等于在真实 nav2 配置上喂真实点云。
- 接入只需两步（都不改 cod）：
  1. 驱动输出话题/帧对齐：让点云出现在 `/livox/lidar_filtered`，且 `frame_id` 在 TF 里有链（loopback 下最省事是直接发 `base_link`，或补一条静态 TF）。
  2. 若走 Gazebo 仿真，则需要 `gt_sim_params.yaml` 的副本（把障碍源换成 PointCloud2）+ 本仓库 `jt128/launch/jt128_sim_hil.launch.py`（含帧桥与时间戳回退）。
- 真机路线最干净的做法：让驱动**顶替 Livox** —— 出 `/livox/lidar`（frame `livox_frame`），下游 `cpp_lidar_filter`→`small_point_lio`→`/livox/lidar_filtered`→nav2 全部不用改。

---

# 8. 深挖补充（并行子调查结果）

## 8.1 cod_sim / cod_bringup 补充（子调查 #1）

**新增坑（我第一轮没发现的）**：

1. **`world` 是个死参数**：`fzsd_sim_launch.py:39-41` 声明了却从未使用；世界与出生位姿实际由 `fzsd_gazebo_launch.py:37-41` 重新读 `gz_world.yaml` 决定。传 `world:=rmul_2024` 会**静默**沿用 rmul_2025 的位姿，与 `init_x/y`(`fzsd_sim_launch.py:57-59`) 不匹配 → 机器人在 map 里位置错。
2. **`use_sim_time` 基本是摆设**：`loopback_sim_launch.py:72,84`、`singlenav_launch.py:123,133`、`multiplenav_launch.py:141` 在 include navigation/localization 时**写死 "false"**，所以从命令行传 `use_sim_time:=true` 不生效（只有 `fzsd_sim_launch.py:146,148` 老老实实传了 `'True'`）。
3. **外部一键脚本才是 fzsd2025 依赖的来源**：`launch.sh` 会 source `$FZSD_DIR`（默认 `~/fzsd2025`）再启 fzsd_sim —— 之前"仿真需要 fzsd2025"的印象出自这里，而不是 `fzsd_sim_launch.py` 本身。
4. **真机缺 TF 提供者**：`singlenav_launch.py` / `multiplenav_launch.py` 都**不起 robot_state_publisher**，而 `small_point_lio/config/mid360.yaml:6` 要 `livox_frame`、`multiplenav` 的 `pointcloud_to_laserscan` 要 `target_frame: base_link`、slam 要 `base_frame: base_link` —— 这些外参链目前**没有发布者**（只有直接跑 `small_point_lio/launch/small_point_lio.launch.py` 才会发 `base_link→livox_frame` 静态 TF）。**这是真机接入 JT128 时必须先补的一环**。
5. **`multiplenav` 同时起 slam_toolbox 与静态 map→odom**（`:85` vs `:95`），而 slam 参数 `transform_publish_period: 0.0`（`mapper_params_online_async.yaml:28`，注释写明"0 = 从不发布"）→ **地图永不出 map→odom**，定位实际靠静态恒等 TF 顶着。
6. **两套参数体系互斥**：`gt_sim_params.yaml:177` 用 `gimbal_yaw_fake`，`singlenav2_params.yaml:184` 用 `base_link_fake`；两个 `nav2_container` 同名，**fzsd_sim 与 loopback 绝对不能同时跑**（我上次就是踩了这个：两个仿真互相打架）。
7. **孤儿配置**：`cod_sim/config/fzsd_sim_params.yaml`（只被 `setup.py:17` 安装，无 launch 引用）、`cod_bringup/params/reality_mppi_nav2_params.yaml`（无引用）；`cod_sim/vendor/priest/params/costmap_common_params.yaml` 还是 **ROS1 遗留**（`costmap_2d::ObstacleLayer`）。
8. **默认值指向不存在的文件**：`singlenav_launch.py:23` 默认 `params/mapper_params_async.yaml` 不存在（只有 `mapper_params_online_async.yaml`），且该参数无人消费。
9. **`cod_sim` 自定义节点的脆弱点**：
   - `gt_odom.py:27` 默认 `child_frame_id: gimbal_yaw`，但仿真必须靠 launch 传参改成 `base_footprint`（`fzsd_sim_launch.py:89`）—— 忘记传就整条 TF 断。
   - `tf_forward.py:18-19` 话题名写死 `/red_standard_robot1/...`，换机器人名即失效。
   - `pcd_publisher.py:19-21` **硬编码 32 字节/点**，换 PCD 格式会解析错位，且整文件一次性读入内存。
   - `priest_bridge.py:221-223` 用 `lookup_transform('odom', cloud_frame)`，异常时**静默 return 丢帧**（不打印），排查困难；且依赖 JAX/GPU。
   - `fake_odom.py:40-42` 同时发 `odom` 和 `Odometry` 两个话题，是为迁就 `fake_vel_transform.cpp:16` 的默认值 `odom_topic: Odometry` 的旁路补丁。
10. **`auto_save_map.launch.py:12-15`** 也硬编码 `/home/cod-sentry/dyx_ws/...`。
11. **README 已过时**：`cod_sim/README.md` 描述的架构仍是 point_lio + small_gicp 版，与现行 `fzsd_sim_launch.py`（gt_odom + rplidar scan + 静态定位）不符。

**对 HIL 的直接结论（强化 §7）**：
- loopback 与真机共用 `singlenav2_params.yaml` + `base_link_fake` 帧体系 → **HIL 走 loopback 就等于在真机配置上验证**，只需保证点云帧（`livox_frame` 或直接 `base_link`）在 TF 里有链。
- 走 Gazebo 仿真那条路，除参数副本外还得处理 `gimbal_yaw_fake` 与可动关节链（依赖 joint_states 存活）。

## 8.2 定位 / 决策 / 裁判 补充（子调查 #2 + 我复核）

### small_point_lio（真机定位的唯一来源）

- 是什么：Point-LIO 的高速重写移植（`small_point_lio/README.md:3`，MIT），核心 ESKF/iVox 自研。
- 发布：`/Odometry`(`src/small_point_lio_node.cpp:26`)、`/cloud_registered`(`:27,127`)、TF `odom→base_link`(`:59-62`)；**不发任何 path 话题**。
- **硬依赖 TF**：回调里先 `lookupTransform(lidar_frame, "base_link")`，失败**直接 return**（于是"没有里程计"，且不报错）→ `small_point_lio_node.cpp:64-69,108-112`。
- ⚠️ **它的静态 TF 定义了却从未发布**：`launch/small_point_lio.launch.py:24-45` 造了 `base_link→livox_frame` 静态 TF，但 `:47` 只 `return LaunchDescription([small_point_lio_node])`。真机 `singlenav_launch.py` 直接用 Node 起 LIO，也不含这个 TF → **`base_link→livox_frame` 全仓库无人发布**。

### 🔴 点云字段要求——这条直接决定 JT128 能不能喂给 LIO（我逐行复核）

`src/lidar_adapter/livox_pointcloud2.h:25-37` 读取的字段是**写死的四个名**：

```cpp
PointCloud2ConstIterator<float>   out_x(msg, "x");      // 还要 y、z
PointCloud2ConstIterator<uint8_t> out_tag(msg, "tag");   // ← 必须有 tag
PointCloud2ConstIterator<double>  out_timestamp(msg, "timestamp");
if ((*out_tag & 0b00111111) == 0b00000000) {             // tag&0x3F==0 才收
    new_point.timestamp = *out_timestamp * 1e-9;         // ← timestamp 按纳秒解释
```

而 Livox 驱动给的字段（`fzsd_vendor/livox_ros_driver2/src/lddc.cpp:249-273`）正好是：
`x, y, z, intensity(float32) + **tag**(uint8) + line(uint8) + timestamp(float64)`。

**禾赛 JT128 驱动给的是**：`x, y, z, intensity(float32) + ring(uint16) + timestamp(float64)` —— **没有 `tag`，且是 `ring` 不是 `line`**。

➡️ 结论：**JT128 的点云无法直接喂给 `small_point_lio`**（`lidar_type: livox_pointcloud2` 会取不到 `tag` 字段）。要接真机定位，必须先做其中之一：
1. 写个转换节点：补 `tag=0`(uint8)、把 `timestamp` 转成**纳秒**、`ring→line`，再发出去（最干净，不动 LIO 代码）；
2. 或者给 `small_point_lio` 加一个"hesai_pointcloud2"适配器（按 `uidp1_4` 的字段布局解析，时间戳单位按禾赛定义换算）；
3. 并**补上 `base_link→livox_frame`（或你选的雷达帧）的静态 TF** —— 否则 LIO 连一帧都不处理（见上一条）。

其它与时间有关的坑：
- 适配器时间戳单位**三套硬编码**：`livox_pointcloud2` ×1e-9（纳秒）、`custom_mid360_driver` 不换算（秒）、`unitree_lidar` 用 header.stamp + 相对秒。配错量级就是 1e9，表现为"机器人不动"或"瞬间漂飞"（`livox_pointcloud2.h:37`、`custom_mid360_driver.h:35`、`unitree_lidar.h:37`）。
- `livox_custom_msg` 需要编译宏 `HAVE_LIVOX_DRIVER`，否则节点直接 shutdown（`small_point_lio_node.cpp:178-185`）。
- **odom 的 twist 被注释掉了**（`small_point_lio_node.cpp:90-96`）→ nav2/MPPI 从 odom 读到的速度恒为 0。

### 决策层（cod_decision_bt）与裁判系统

- 树：`ReactiveFallback MainDecision` 三分支 —— ①非 RUNNING 就 `StopRobot`；②血量低 → `SetRetreatGoal` → `NavigateToPose`；③`Patrol`：`PatrolNextWaypoint` → `NavigateToPose`（`behavior_trees/decision_tree.xml:2-26`）。
- **仿真与真机的树 XML 逐字相同**，差异全在两份 yaml：`config/decision_tree_params.yaml`（仿真：server `/navigate_to_pose`、`stop_cmd_vel_topic:/cmd_vel`）vs `config/reality/decision_tree_reality.yaml`（真机：`/red_standard_robot1/navigate_to_pose`、`/red_standard_robot1/cmd_vel`）。
- 裁判模拟器：状态机 PREPARATION(60s)→SELF_CHECK(5s)→RUNNING(300s)→GAME_OVER，1 Hz 自动推进（`cod_referee_simulator/src/referee_simulator_node.cpp:60-111`）；**默认 60 秒后自动开赛**，GAME_OVER 时 `rclcpp::shutdown()`(`:97`) → 之后无裁判消息，树永久停车。
- 决策链只用到 `GameStatus.stage`、`RobotStatus.remain_hp/max_hp`；`Decision.msg` 全仓库无引用（死接口）。

### 坑（精选，均带行号出处）

1. **停赛停不下来**：`StopRobot` 只发 0 速，决策层**从不 cancel 已发的 NavigateToPose**（`decision_tree_node.cpp` 无 action cancel 逻辑）→ nav2 仍按旧 goal 发速度。
2. **`patrol_use_spawn_pose: true` 必挂**：`spawn_pose` 黑板键从未被写入（`decision_tree_node.cpp:79-98`），而 `patrol_next_waypoint_action.cpp:71,126-128` 会读它；两份 yaml 恰好都是 false 才没暴露。
3. **collector 一启动就发空列表**（`patrol_collector_node.cpp:74`）→ 决策树立刻进 live 模式、忽略 yaml 里的静态航点；真机 launch 默认 `start_collector:=true` 且 `patrol_waypoints: ""` → 不在 RViz 点过点就永远只停车/回补给。
4. **`multiplenav` 同时起 slam_toolbox 与静态 map→odom**，而 slam 的 `transform_publish_period: 0.0`（`mapper_params_online_async.yaml:28`）= 从不发布 → 实际靠静态恒等 TF 顶着建图。
5. **两套真机栈互斥**：cod 栈（`livox_frame` / `/Odometry` / `base_link_fake`）vs fzsd reality 栈（`front_mid360` / `aft_mapped_to_init` / `gimbal_yaw_fake`）。`small_point_lio` **不发** `aft_mapped_to_init`，接 fzsd 栈就没输入；`map_reality.sh:49` 说明**实车实际跑的是 fzsd 那套**。
6. **点云消费者不一致**：costmap 吃 `/livox/lidar_filtered`，而 LIO 与 multiplenav 的 pointcloud_to_laserscan 吃**未过滤**的 `/livox/lidar` → 自车体点云进了 LIO 和 scan。
7. 决策节点 `use_sim_time` 未设（`decision_tree.launch.py:13-19`）→ 仿真里它用墙钟，goal 时间戳与 `/clock` 不符。

### 对 JT128 接入的结论修正（重要）

我在 §7 写的"让驱动顶替 Livox 出 `/livox/lidar` 就全都不用改"——**对 costmap 成立，对 LIO 不成立**。修正后的真机接入清单：

1. 点云转换节点：`x,y,z,intensity` 保留 → **补 `tag=0`(uint8)、`line=(ring&0xFF)`、把 `timestamp` 转成纳秒**；
2. 补 `base_link→<雷达帧>` 静态 TF（现在全仓库没人发）；
3. 里程计/IMU：LIO 需要 `/livox/imu`（JT128 数据尾里有 IMU 字段，但驱动**不发布** `/livox/imu` 话题！→ 要么用 JT128 的 IMU 自己拼 Imu 消息，要么保留原 Livox 的 IMU 做 LIO 的 IMU 源，此时雷达点云换 JT128、IMU 仍来自 Mid-360——**混用两套硬件是可行但需注意外参**）；
4. 若走 fzsd reality 那套栈（实车实际用的），还要对齐 `front_mid360` 帧名与 `aft_mapped_to_init` 话题约定。

## 8.3 运动控制 / 插件层补充（子调查 #3）

**控制器链路（只有一条）**：`controller_plugins: ["FollowPath"]` → `FollowPath.plugin = goal_approach_controller::GoalApproachController` → 它用**同一个名字**内部加载 `nav2_mppi_controller::MPPIController`（`goal_approach_controller.cpp:60-63`）。所以 **MPPI 的参数必须写在 `FollowPath.*` 下**才生效。
- `pb_omni_pid_pursuit_controller`（全向 PID 追击）**本仓三份 yaml 都没用**，只有 fzsd vendor 的 reality/simulation 参数选了它；`reality_mppi_nav2_params.yaml` 是孤儿。
- 它的 pluginlib XML **没有 `name` 属性**（`pb_omni_pid_pursuit_controller.xml:3`），只能写全限定类名（pluginlib 回退"查找名=类名"）。

⚠️ **`goal_approach_controller` 有个安全性问题**（`goal_approach_controller.cpp:109-123`）：它把 costmap `global_frame` 下的 `dx/dy` **直接当成 body frame 的 `twist.linear.x/y`**，且不看 `pose.orientation` —— 只有在 yaw≈0 时才朝目标；并且在 `direct_approach_distance`（实配 2.0 m）以内**完全绕过 MPPI 避障**并强制 `angular.z=0`，存在直线撞障风险。

**最关键的两条硬伤**：

1. 🔴 **`pb_nav2_costmap_2d::IntensityObstacleLayer` 在本仓库不存在**：`reality_mppi_nav2_params.yaml:440,481` 与 vendored `pb2025_nav_bringup/config/reality/nav2_params.yaml:380,421` 都引用它，但本仓 `pb_nav2_plugins/costmap_plugins.xml:3` 只导出 `IntensityVoxelLayer` —— **该类只存在于 `~/fzsd2025` 那份副本里**。不 source fzsd2025 时 costmap 层直接加载失败；两份都 source 时，同名类由 `std::map::insert` **静默取先到者**（`class_loader_imp.hpp:766`）。→ 这解释了"为什么有些路径离不开 fzsd2025"，也是换机时的隐形雷。
2. 🔴 **`cpp_lidar_filter` 的降采样确实是死的**（`filter_node.cpp:106-114` 整段注释），launch 传的 `leaf_size=0.05` 无效 → 全密度点云直灌 STVL 体素层；且 CropBox 是全量 PCL 拷贝（`:83-104`）。**这是整条链路最大的负载隐患**（JT128 的 11~23 万点/帧会更明显）。

**其它要点**：
- `fake_vel_transform`：订 `Odometry`(depth10) + `cmd_vel`(depth1)，发 `aft_cmd_vel`(**depth1**)；**把角速度强制成 `spin_speed_`**（`fake_vel_transform.cpp:55-65`），默认 0，且 yaml 里的 `cmd_spin_topic`/`init_spin_speed` 节点从未声明 → **静默失效**。`cmd_vel` 订阅 depth=1 对 20~50 Hz 控制器偏小，易丢帧。
- `pointcloud_to_laserscan`：**只有 `scan` 有订阅者时才订阅 `cloud_in`（懒订阅）**（`pointcloud_to_laserscan_node.cpp:121-123`）→ HIL 调试时"没点云"可能只是没人订阅 scan。
- `cod_serial_ul26`（串口）：**每帧开关串口 + 15 条 INFO 日志**（`cod_serial.cpp:43-54`），20~50 Hz 必淹日志；包 15 B 但 `packet[14]` 未初始化；**只发 vx/vy/(vz=linear.z)，不下发角速度**（`:24,31`）。
- `filter_node.cpp:56` 的 crop_box marker frame 硬编码 `base_link`，而 CropBox 实际作用在点云自身 frame → 机身挖除区可能偏。

## 8.4 fzsd_vendor 层补充（子调查 #4）

**版本与来源**（`package.xml` 的 version + 上游 commit）：

| 包 | version | 上游 |
|---|---|---|
| rmu_gazebo_simulator | 1.0.0 | SMBU-PolarBear/rmu_gazebo_simulator @ e09b39f |
| rmoss_core / rmoss_gazebo / rmoss_gz_resources / rmoss_interfaces | 0.9~1.0 | robomaster-oss/* @ 8209ec5 / e3fc585 / e7d0adf / 424f50c |
| sdformat_tools | 0.0.1 | gezp/sdformat_tools @ 47c2d1a |
| fzsd2025_robot_description | 1.0.0 | SMBU/pb2025_robot_description @ 2543d91（**本地改了包名**，README 标题还写着 pb2025） |
| livox_ros_driver2 | 1.1.0（tag） | SMBU fork @ 524e14a（**CHANGELOG 已到 1.2.4 → 版本号不可信，以 commit 为准**） |
| pb2025_nav_bringup | 1.2.0 | SMBU/pb2025_sentry_nav @ 7089d58（本地裁剪过） |
| vision_interfaces / nav2_command_handler | 1.0.0 / 0.0.0 | 前者死重、后者仅被 vendored 的 fzsd launch 引用 |

**机器人模型（SDF 1.7 + xmacro，不是 URDF/xacro）** —— 与我实测的 TF 完全对上：

| 关节 | 类型 | pose | 出处 |
|---|---|---|---|
| base_footprint→chassis | fixed | `0 0 0.076` | `rmua19_standard_robot.def.xmacro:11-14` |
| chassis→gimbal_yaw | revolute(z) | `0 0 0.1376` | `:55-58` |
| gimbal_yaw→gimbal_pitch | revolute(y) | `0 0.004 0.16`，限位 -0.785~0.565 | `:84-92` |
| **gimbal_yaw→front_mid360** | **fixed** | **`0.04541 0.19059 0.28 -0.7854 0 0`（roll -45°）** | `fzsd2025_sentry_robot.sdf.xmacro:21` |
| chassis→front_rplidar_a2 | fixed | `0.155 0 0.1` | `:18` |

> 实测交叉验证：我用 tf2 量到的 `front_mid360` 相对位姿是 `0.045, 0.191, 0.28` + roll `-0.785` —— 与 SDF 数值一致 ✓。
> 仿真里那颗 mid360 的垂直 FOV 是 **-7°~+52°**（非对称，`mid360/model.sdf.xmacro:44-53`），且**装在随动云台上并 roll -45°**；cod 仿真只用 rplidar 的 scan，所以这点被掩盖。

**世界/桥接**：`gz_world.yaml:1` 世界 `rmul_2025`，**两台车** —— red `x=-5,y=3,z=0.28,yaw=0`、blue `x=4.8,y=-3.5,z=0.28,yaw=3.14`。桥接 7 条全 GZ→ROS（odom/joint_state/image/scan/点云/imu/camera_info）；`/clock` 也在桥里。URDF 由 `xmacro4sdf → sdformat_tools.urdf_generator`（纯 Python `sdf2urdf.py:159`）生成。

**与 `~/fzsd2025` 的关系（修正 §8.1 的说法）**：gazebo 三件套 + robot_description + livox 驱动 + vision_interfaces **逐字节相同**；但 **`pb2025_nav_bringup` 有差异**（少了 `pcd/`、package.xml 依赖被裁、CMakeLists 改成纯资源包），`nav2_command_handler` 少一个空目录。根脚本先 source `$FZSD_DIR/install` 再 source 本仓 install → **后 source 者覆盖，vendored 那份生效**（所以仿真不依赖外部 fzsd2025）。
**例外**：`IntensityObstacleLayer` 这个类只在 fzsd2025 那份 `pb_nav2_plugins` 里（见 §8.3 坑 1）。

**新增坑**：
- **`set_performer` 服务根本不存在**：5 个 world sdf 里 `level` 出现 0 次（`grep -c level resource/worlds/*.sdf` 全 0），而 `fzsd_gazebo_launch.py:117-126` 仍去调 `/world/default/level/set_performer` → **静默失败**，别把它当成功判据。
- **vendored `spawn_robots.launch.py:75-82` 有负数位姿 gflags bug** → vendored 的 `bringup_sim.launch.py` 对 rmul_2025 不可信；cod 侧用 `-x=` 绕过了（`fzsd_gazebo_launch.py:84-87`）。
- **换世界即踩坑**：`gz_world.yaml` 里 rmul_2024 的 `z_pose=1.2`，而静态 TF 固定 `z=0.28`、`init_x/init_y` 也是按 rmul_2025 写死（`fzsd_sim_launch.py:17-18` 注释自认三处必须同步）。
- **vendored pb2025 的 launch 仍是实车版**：`rm_sentry_reality_launch.py:168` 需要 point_lio/small_gicp/loam_interface/terrain_analysis（本仓库没有），默认 `prior_pcd_file` 指向已被删掉的 `pcd/reality/*.pcd` → 误跑必报找不到文件。
- **【分歧裁决】红车 TF 到底发在全局还是命名空间？** 两份子调查结论相反（一份据 `tf_forward` 的存在推断"在命名空间"，一份据注释推断"注释与实现矛盾"）。**以实测为准**：我早先用 `ros2 topic echo /tf_static --once` 抓到的帧对是**不带命名空间**的 `base_footprint→chassis`、`chassis→front_rplidar_a2`、`gimbal_yaw→front_mid360` —— 说明**红车 RSP 确实直发全局 `/tf(_static)`**（对应 `fzsd_gazebo_launch.py:101` 的 `rsp_remap = []`），注释是对的；`tf_forward` 对红车实际是**空转**（它订阅 `/red_standard_robot1/tf`，红车没往那儿发），保留它主要为兼容/历史原因。**排 TF 问题以 `ros2 topic echo /tf_static` 的实测帧名为准**。
