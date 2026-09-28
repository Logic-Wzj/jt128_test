# JT128 现场测试工具包

给"小电脑 / 机器人主机"用的禾赛 JT128 雷达测试与接入脚本（ROS 2 Humble / Ubuntu 22.04）。

## 小电脑三步

```bash
git clone <本仓库地址> jt128_test && cd jt128_test   # 目录叫什么都行
./jt128/port/deps_check.sh                            # 1. 依赖自检（应无 ❌）
./jt128/port/port_setup.sh "$PWD"                     # 2. 部署（装驱动+编译+写 shell 入口）
source ~/.bashrc && ./jt128/launch.sh radar            # 3. 配网+探包+启动驱动
```

## 内容

- `jt128/` 工具套件：`launch.sh`（一键入口）、`jt128_check.py`（裸 UDP 收包自检）、`memwatch.py`（内存/CPU/FD 监控）、
  `jt128_verify.sh`（验收）、`port/`（移植部署）、`sim/`（仿真 HIL 用）
- `jt128_sim_relay/` 仿真中继（`/lidar_points` → `/jt128/points`）
- `jt128_livox_compat/` 禾赛点云 → Livox 格式兼容层（喂给 small_point_lio）

- `HesaiLidar_ROS_2.0/` 禾赛官方 ROS2 驱动（**Modified BSD-3**，LICENSE 随包保留，上游 commit `e7e112f` / tag `v2.0.12`）

## 文档（都在 `jt128/`）

`上手步骤.md`、`接上雷达后.md`、`完整验收清单.md`、`GitHub传包.md`、`小电脑部署卡.md`
