#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hdl_loc.launch.py —— 在 fzsd 实车栈里起 NDT 重定位（A/B 的 B 方案）

起两个进程：
  1. cloud_to_base   : /livox/lidar(front_mid360)  -> /livox/lidar_base(base_footprint)
                       参数见同目录 cloud_to_base.yaml
  2. pcl_localization: NDT_OMP 配准 -> 广播 map->odom
                       参数见同目录 pcl_loc_mid360.yaml

相对上游 launch 的改动（都是必须的）：
  * 删掉两个 demo 静态 TF（base_link->velodyne / base_link->imu_link）——
    fzsd 栈用的是 base_footprint，这两个会把 TF 树搞乱。
  * remap 改成 velodyne_points:=/livox/lidar_base：
    节点源码里订阅的话题名写死是 velodyne_points，上游那个 /cloud:=... 其实是空操作。

用法：
  ros2 launch ~/jt128/fzsd_loc/hdl_loc.launch.py
  ros2 launch ~/jt128/fzsd_loc/hdl_loc.launch.py params_file:=/abs/path.yaml
  ros2 launch ~/jt128/fzsd_loc/hdl_loc.launch.py start_relay:=false

起完后给初始位姿：RViz 固定坐标系选 map，用 "2D Pose Estimate" 点一下
（和 GICP 共用同一个 /initialpose，A/B 才公平）。
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PARAMS = os.path.join(HERE, "pcl_loc_mid360.yaml")
RELAY_SCRIPT = os.path.join(HERE, "cloud_to_base.py")
RELAY_PARAMS = os.path.join(HERE, "cloud_to_base.yaml")


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    start_relay = LaunchConfiguration("start_relay")

    args = [
        DeclareLaunchArgument("params_file", default_value=DEFAULT_PARAMS,
                              description="NDT 参数文件"),
        DeclareLaunchArgument("start_relay", default_value="true",
                              description="是否顺带起 cloud_to_base 点云中继"),
    ]

    # ---- 1. 点云中继：把点云搬到 base_footprint ----
    relay = ExecuteProcess(
        cmd=[sys.executable, RELAY_SCRIPT,
             "--ros-args", "--params-file", RELAY_PARAMS],
        name="cloud_to_base",
        output="screen",
        condition=IfCondition(start_relay),
    )

    # ---- 2. NDT 重定位 ----
    pcl_localization = LifecycleNode(
        name="pcl_localization",
        namespace="",
        package="pcl_localization_ros2",
        executable="pcl_localization_node",
        parameters=[params_file],
        remappings=[
            ("velodyne_points", "/livox/lidar_base"),   # 源码里写死的话题名
            ("initialpose", "/initialpose"),
            ("odom", "/odometry"),
        ],
        output="screen",
    )

    to_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(pcl_localization),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    ))

    on_unconfigured = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=pcl_localization,
        goal_state="unconfigured",
        entities=[
            LogInfo(msg="[loc] unconfigured -> configure"),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(pcl_localization),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )),
        ],
    ))

    on_inactive = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=pcl_localization,
        start_state="configuring",
        goal_state="inactive",
        entities=[
            LogInfo(msg="[loc] inactive -> activate"),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(pcl_localization),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )),
        ],
    ))

    ld = LaunchDescription(args)
    ld.add_action(relay)
    ld.add_action(on_unconfigured)
    ld.add_action(on_inactive)
    ld.add_action(pcl_localization)
    ld.add_action(to_configure)
    return ld
