"""
JT128 -> Livox Mid-360 兼容层启动文件

作用：把 JT128 的点云/IMU 变成 Mid-360 的样子，接到 cod 那套真机导航栈上：

    hesai_ros_driver  ──/lidar_points──> jt128_livox_compat ──> /livox/lidar   (frame=livox_frame)
                      └─/lidar_imu────>        └──────────────> /livox/imu
                                                               +
                                      静态 TF: base_link -> livox_frame（LIO 必需）

之后启动你原来的 cod_bringup/singlenav_launch.py 即可（它会起 cpp_lidar_filter、
small_point_lio、nav2），全部不用改。

用法：
    # 只起适配层（驱动已在别处跑）
    ros2 launch jt128_livox_compat compat.launch.py
    # 连驱动一起起（雷达已在 /lidar_points 配置下）
    ros2 launch jt128_livox_compat compat.launch.py start_driver:=true
    # 雷达实际装的位置不为零时，补上偏移（LIO 要用它把点云变到 base_link）
    ros2 launch jt128_livox_compat compat.launch.py x:=0.05 z:=0.25

注意：
  - **IMU 频率**：LIO 靠 IMU 递推，Mid-360 是 200 Hz。JT128 的 IMU 频率要实测
    （ros2 topic hz /lidar_imu）；若明显偏低，LIO 会漂/抖，此时建议退回"点云用 JT128、
    IMU 仍用 Mid-360"的混装方案（把 relay_imu:=false，另行 remap Mid-360 的 IMU）。
  - 雷达的实际安装外参以真机 URDF 为准，这里默认全 0（等价于与 base_link 重合）。
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
        # 话题与坐标系
        DeclareLaunchArgument('in_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('out_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('target_frame', default_value='livox_frame',
                              description='LIO 配置里 lidar_frame 用的帧名'),
        # IMU
        DeclareLaunchArgument('relay_imu', default_value='true',
                              description='把 /lidar_imu 转发成 /livox/imu；'
                                          'false 则用外部 IMU（如 Mid-360）'),
        DeclareLaunchArgument('in_imu_topic', default_value='/lidar_imu'),
        DeclareLaunchArgument('out_imu_topic', default_value='/livox/imu'),
        # 静态 TF
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # 其它
        DeclareLaunchArgument('stamp_offset_ms', default_value='0'),
        DeclareLaunchArgument('timestamp_scale', default_value='1e9',
                              description='逐点 timestamp 换算系数：禾赛驱动是秒，'
                                          'small_point_lio 按纳秒解释（×1e-9），故默认 1e9；设 1 不换算'),
        DeclareLaunchArgument('qos_reliable', default_value='false'),
        DeclareLaunchArgument('start_driver', default_value='false'),
        DeclareLaunchArgument('config_path', default_value=default_config),
    ]

    compat = Node(
        package='jt128_livox_compat',
        executable='jt128_livox_compat_node',
        name='jt128_livox_compat',
        output='screen',
        parameters=[{
            'in_topic': LaunchConfiguration('in_topic'),
            'out_topic': LaunchConfiguration('out_topic'),
            'target_frame': LaunchConfiguration('target_frame'),
            'relay_imu': LaunchConfiguration('relay_imu'),
            'in_imu_topic': LaunchConfiguration('in_imu_topic'),
            'out_imu_topic': LaunchConfiguration('out_imu_topic'),
            'stamp_offset_ms': LaunchConfiguration('stamp_offset_ms'),
            'timestamp_scale': LaunchConfiguration('timestamp_scale'),
            'qos_reliable': LaunchConfiguration('qos_reliable'),
        }],
    )

    # LIO 回调里会 lookupTransform(lidar_frame, "base_link")，拿不到就静默不发里程计，
    # 而这个 TF 在 cod 仓库里没有任何节点发布 —— 由这里补上。
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        output='log',
        condition=IfCondition(LaunchConfiguration('publish_tf')),
        arguments=[
            '--x', LaunchConfiguration('x'),
            '--y', LaunchConfiguration('y'),
            '--z', LaunchConfiguration('z'),
            '--roll', LaunchConfiguration('roll'),
            '--pitch', LaunchConfiguration('pitch'),
            '--yaw', LaunchConfiguration('yaw'),
            '--frame-id', LaunchConfiguration('base_frame'),
            '--child-frame-id', LaunchConfiguration('target_frame'),
        ],
    )

    driver = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        name='hesai_ros_driver_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_driver')),
        parameters=[{'config_path': LaunchConfiguration('config_path')}],
    )

    return LaunchDescription(args + [driver, compat, static_tf])
