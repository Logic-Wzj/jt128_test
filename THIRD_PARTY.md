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
