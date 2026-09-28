"""
JT128 → Gazebo 仿真 HIL 启动文件（真实雷达接入仿真）

数据流：
    真雷达 ──UDP──> hesai_ros_driver ──/lidar_points──> jt128_sim_relay (C++)
                                                            │ ① frame_id: front_jt128 -> front_mid360
                                                            │ ② 时间戳: 绝对时间 -> 仿真时钟
                                                            │ ③ 可选抽稀 stride / max_points
                                                            v
                                                     /jt128/points ──> 仿真 nav2 local_costmap

为什么中继是 C++（不是 Python）：
    rclpy 反序列化 1.8 MB 的 uint8[] 是逐元素的，实测 Python 中继只能吃 1~2 帧/s；
    真雷达单回波就是 10 Hz × 1.84 MB，Python 直接堵死。C++ 版实测 10.0 Hz 零丢帧。

为什么必须设置 Fast DDS profile（本 launch 已自动 SetEnvironmentVariable）：
    1.84 MB/帧 > 本机 socket 缓冲上限 208 KB，也 > Fast DDS 默认共享内存段 536 KB，
    消息只能退回 UDP 分片重组 → 大量丢帧（现象：发布端 10 Hz，订阅端只有 1~2 Hz）。
    用 64 MB 共享内存段后可满速。
    ⚠️ 仿真那一侧（Gazebo + nav2）也必须带上同一个环境变量，否则 costmap 收不到：
        export FASTRTPS_DEFAULT_PROFILES_FILE=<本目录>/sim/fastdds_large_msg.xml

仿真侧命令（另开终端；FASTRTPS 变量由 jt128.zsh 的 jt128sim 自动注入）：
    ros2 launch cod_sim fzsd_sim_launch.py \
        params_file:=<本目录>/sim/gt_sim_jt128_params.yaml

    注：仿真所需的 rmu_gazebo_simulator / pb2025_nav_bringup / fzsd2025_robot_description
    都由 cod 仓库的 src/fzsd_vendor/ 提供并编译进 cod 工作区，**不需要**外部 ~/fzsd2025
    （fzsd_sim_launch.py 开头"前置：source ~/fzsd2025/..."那句注释是过时的）。

本文件用法：
    source ~/jt128/jt128.zsh        # 或手动 source 两个 setup.zsh（cod + hesai_ws）
    ros2 launch ~/jt128/launch/jt128_sim_hil.launch.py                 # 真雷达
    ros2 launch ~/jt128/launch/jt128_sim_hil.launch.py source:=fake    # 无雷达干跑（合成房间+柱子）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # start_driver:=false（只起中继）时并不需要禾赛包，所以解析失败不能让它整个启动失败
    try:
        default_config = os.path.join(
            get_package_share_directory('hesai_ros_driver'), 'config', 'config.yaml')
    except Exception:
        default_config = os.path.join(
            os.path.expanduser('~'), 'HesaiLidar_ROS_2.0', 'config', 'config.yaml')
    # 按启动文件自身位置推导目录（不写死 ~/jt128，整个目录拷到别处也能用）
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    fake_py = os.path.join(pkg_root, 'sim', 'jt128_fake_cloud.py')
    default_profile = os.path.join(pkg_root, 'sim', 'fastdds_large_msg.xml')

    source = LaunchConfiguration('source')
    use_fake = PythonExpression(["'", source, "' == 'fake'"])
    use_real_driver = PythonExpression(
        ["'", source, "' == 'real' and '", LaunchConfiguration('start_driver'), "' == 'true'"])

    args = [
        DeclareLaunchArgument('source', default_value='real',
                              description="real=真雷达驱动 | fake=合成点云干跑"),
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument('in_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('out_topic', default_value='/jt128/points'),
        DeclareLaunchArgument('target_frame', default_value='front_mid360',
                              description='仿真 TF 里存在的雷达帧'),
        DeclareLaunchArgument('stride', default_value='1',
                              description='每 N 个点保留 1 个（costmap 跟不上时用 4）'),
        DeclareLaunchArgument('max_points', default_value='0',
                              description='>0 时按帧均匀抽到该点数'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_driver', default_value='true',
                              description='false=驱动已在别处跑，只起中继'),
        DeclareLaunchArgument('stamp_offset_ms', default_value='150',
                              description='点云时间戳回退(ms)。必须>0：中继用 now() 打戳会比 '
                                          'Gazebo 的 TF 快十几毫秒，nav2 会判"外推到未来"丢帧'),
        DeclareLaunchArgument('bridge_gimbal_tf', default_value='true',
                              description='补 chassis->gimbal_yaw 这一环。仿真的全局 TF 树里缺它，'
                                          '导致 front_mid360 与 odom 不连通（两棵断树）'),
        DeclareLaunchArgument('gimbal_z', default_value='0.1376',
                              description='chassis->gimbal_yaw 的高度偏移，取自 fzsd SDF'),
        DeclareLaunchArgument('fastdds_profile', default_value=default_profile,
                              description='Fast DDS 大消息 profile（共享内存段 64 MB）'),
    ]

    # 关键：没有这个 profile，1.84 MB 的点云会在 DDS 层被丢到只剩 1~2 Hz
    set_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE', LaunchConfiguration('fastdds_profile'))

    driver = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node',
        output='screen',
        condition=IfCondition(use_real_driver),
        parameters=[{'config_path': LaunchConfiguration('config_path')}],
    )

    # 干跑：合成点云发到 in_topic，再走同一个 C++ 中继，链路与真雷达完全一致
    fake = ExecuteProcess(
        cmd=['python3', fake_py, '--ros-args',
             '-p', ['topic:=', LaunchConfiguration('in_topic')],
             '-p', ['frame_id:=', LaunchConfiguration('target_frame')],
             '-p', ['use_sim_time:=', LaunchConfiguration('use_sim_time')]],
        output='screen',
        condition=IfCondition(use_fake),
    )

    relay = Node(
        package='jt128_sim_relay',
        executable='jt128_sim_relay_node',
        name='jt128_sim_relay',
        output='screen',
        parameters=[{
            'in_topic': LaunchConfiguration('in_topic'),
            'out_topic': LaunchConfiguration('out_topic'),
            'target_frame': LaunchConfiguration('target_frame'),
            'stride': LaunchConfiguration('stride'),
            'max_points': LaunchConfiguration('max_points'),
            'stamp_offset_ms': LaunchConfiguration('stamp_offset_ms'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # 仿真的全局 TF 树是断的：odom→base_footprint→chassis 一棵，
    # gimbal_yaw→front_mid360 另一棵，中间缺 chassis→gimbal_yaw（revolute 关节，全局没人发）。
    # 不补这一环，nav2 的 local_costmap 拿不到 front_mid360→odom，会直接丢帧。
    gimbal_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='chassis_to_gimbal_yaw',
        output='log',
        condition=IfCondition(LaunchConfiguration('bridge_gimbal_tf')),
        arguments=[
            '--x', '0', '--y', '0', '--z', LaunchConfiguration('gimbal_z'),
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'chassis', '--child-frame-id', 'gimbal_yaw',
        ],
    )

    return LaunchDescription(args + [set_profile, driver, fake, relay, gimbal_bridge])
