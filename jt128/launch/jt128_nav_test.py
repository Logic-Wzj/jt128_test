"""
JT128 方案 A 集成测试 launch —— 禾赛驱动从独立工作区起，cod 仓库一行不改。

起三个节点：
  1. hesai_ros_driver_node        -> /lidar_points（frame_id=front_jt128，来自 config.yaml）
  2. cpp_lidar_filter/lidar_filter_node -> /livox/lidar_filtered（nav2 原本就在听这个话题）
  3. tf2_ros static_transform_publisher  base_link -> front_jt128（按实际安装位置改偏移）

用法（zsh，你的默认终端；.zshrc 里已 source /opt/ros/humble/setup.zsh）：
    source ~/hesai_ws/install/setup.zsh
    source ~/cod_-rm2026_-navigation/install/setup.zsh
    ros2 launch ~/jt128/launch/jt128_nav_test.py

    或者直接用 zsh 片段里的函数（推荐）：
    source ~/jt128/jt128.zsh && jt128up

bash 用户把 setup.zsh 换成 setup.bash 即可。

常用覆盖：
    ros2 launch ~/jt128/launch/jt128_nav_test.py use_static_tf:=false
    ros2 launch ~/jt128/launch/jt128_nav_test.py z:=0.35        # 雷达安装高度

注意（JT128 相比 Mid-360 的负载差）：
    单回波 10 Hz 每帧 115200 点、双回波 230400 点，是 Mid-360 的数倍。
    cpp_lidar_filter 里的 VoxelGrid 降采样目前是**注释掉的**（filter_node.cpp:106-113），
    也就是只做 CropBox 去车身、不做稀疏化 —— 直接把 11~23 万点丢给 nav2。
    建议先在 LidarUtilities 里把雷达切单回波，或后续给滤波节点打开降采样。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('hesai_ros_driver'), 'config', 'config.yaml')

    args = [
        DeclareLaunchArgument('config_path', default_value=default_config,
                              description='禾赛驱动的 config.yaml'),
        DeclareLaunchArgument('lidar_topic', default_value='/lidar_points',
                              description='驱动输出的原始点云话题'),
        DeclareLaunchArgument('filtered_topic', default_value='/livox/lidar_filtered',
                              description='滤波后话题（nav2 的 observation_sources 用的是这个）'),
        DeclareLaunchArgument('use_static_tf', default_value='true',
                              description='是否发布 base_link -> front_jt128 的静态 TF'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    driver = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': LaunchConfiguration('config_path')}],
    )

    # 输入话题改成 /lidar_points，输出仍是 nav2 认的 /livox/lidar_filtered
    lidar_filter = Node(
        package='cpp_lidar_filter',
        executable='lidar_filter_node',
        name='lidar_filter_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('lidar_topic'),
            'output_topic': LaunchConfiguration('filtered_topic'),
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_front_jt128',
        output='log',
        condition=IfCondition(LaunchConfiguration('use_static_tf')),
        arguments=[
            '--x', LaunchConfiguration('x'),
            '--y', LaunchConfiguration('y'),
            '--z', LaunchConfiguration('z'),
            '--roll', LaunchConfiguration('roll'),
            '--pitch', LaunchConfiguration('pitch'),
            '--yaw', LaunchConfiguration('yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'front_jt128',
        ],
    )

    return LaunchDescription(args + [driver, lidar_filter, static_tf])
