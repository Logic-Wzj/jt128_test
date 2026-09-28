"""
JT128 测试用 launch：只起驱动节点，不启动 rviz2（rviz 的显存/内存会污染资源占用测量）

用法：
    source /opt/ros/humble/setup.bash && source ~/hesai_ws/install/setup.bash
    ros2 launch hesai_ros_driver jt128_test.py                                  # 纯雷达测试：点云发 /lidar_points
    ros2 launch hesai_ros_driver jt128_test.py point_cloud_topic:=/livox/lidar  # 接入 cod：伪装成 Livox

注意（重要）：
    双回波出厂设置下，一帧点云约 230400 点 × 26 B ≈ 6 MB。Fast DDS 默认共享内存段只有
    几百 KB，装不下这么大的消息 → 会退化成 UDP 分片，订阅端只看到 3~4 帧/s（本机实测 3.5 Hz）。
    所以这里默认给节点注入 FASTRTPS_DEFAULT_PROFILES_FILE（64 MB 共享内存段），
    订阅端（rviz2 / jt128_verify.sh）也必须带上同一个环境变量才能跑满 10 Hz。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_DDS_PROFILE = os.environ.get(
    'JT128_DDS_PROFILE',
    os.path.join(os.path.expanduser('~'), 'jt128', 'sim', 'fastdds_large_msg.xml'))


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('hesai_ros_driver'), 'config', 'config.yaml')

    actions = [
        DeclareLaunchArgument(
            'config_path', default_value=default_config,
            description='hesai_ros_driver 的 config.yaml 路径'),
        DeclareLaunchArgument(
            'point_cloud_topic', default_value='/lidar_points',
            description='点云话题重映射目标；默认 /lidar_points（纯雷达测试），接 cod 时传 /livox/lidar'),
        DeclareLaunchArgument(
            'dds_profile', default_value=DEFAULT_DDS_PROFILE,
            description='大消息用的 Fast DDS profile；置空则不注入'),
    ]

    # 大消息（~6 MB/帧）必须走 64 MB 共享内存段，否则只有 3~4 Hz
    dds_profile = LaunchConfiguration('dds_profile')
    if os.path.isfile(DEFAULT_DDS_PROFILE):
        actions.append(SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE', dds_profile))
    else:
        actions.append(LogInfo(msg=(
            '⚠️ 找不到 DDS profile（%s）—— 大点云可能掉到 3~4 Hz' % DEFAULT_DDS_PROFILE)))

    actions.append(Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': LaunchConfiguration('config_path')}],
        remappings=[('/lidar_points', LaunchConfiguration('point_cloud_topic'))],
    ))

    return LaunchDescription(actions)
