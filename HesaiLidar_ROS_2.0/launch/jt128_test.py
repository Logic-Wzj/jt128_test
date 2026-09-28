"""
JT128 测试用 launch：只起驱动节点，不启动 rviz2（rviz 的显存/内存会污染资源占用测量）

用法：
    source /opt/ros/humble/setup.bash && source ~/hesai_ws/install/setup.bash
    ros2 launch hesai_ros_driver jt128_test.py
    ros2 launch hesai_ros_driver jt128_test.py point_cloud_topic:=/lidar_points   # 不改话题
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('hesai_ros_driver'), 'config', 'config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path', default_value=default_config,
            description='hesai_ros_driver 的 config.yaml 路径'),
        DeclareLaunchArgument(
            'point_cloud_topic', default_value='/livox/lidar',
            description='点云话题重映射目标，默认对接现有 cpp_lidar_filter'),
        Node(
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            name='hesai_ros_driver_node',
            output='screen',
            parameters=[{'config_path': LaunchConfiguration('config_path')}],
            remappings=[('/lidar_points', LaunchConfiguration('point_cloud_topic'))],
        ),
    ])
