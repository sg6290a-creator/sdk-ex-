#!/usr/bin/env python3
"""
============================================================================
Navigation Launch File (SLAM / Nav2)
============================================================================

3D LiDAR (Livox MID-360) SLAM + Nav2 navigation stack.
EKF 센서 융합: wheel odom + IMU.

[사용법]
ros2 launch frjoco_bringup navigation.launch.py
ros2 launch frjoco_bringup navigation.launch.py localization:=true

============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('frjoco_bringup')
    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf.yaml')

    nav2_params = os.path.join(
        get_package_share_directory('robot_nav2'), 'config', 'nav2_params.yaml')

    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_ekf', default_value='true'),
        DeclareLaunchArgument('enable_nav2', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ================================================================
    # 1. EKF sensor fusion (wheel odom + IMU → /odometry/filtered)
    # ================================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('enable_ekf')),
    )

    # ================================================================
    # 2. cmd_vel relay: Nav2 /cmd_vel → mobile_sdk diff_drive
    # ================================================================
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        arguments=['/cmd_vel', '/mobile/diff_drive_controller/cmd_vel_unstamped'],
    )

    # ================================================================
    # 3. Nav2 stack (delayed 5s for EKF warmup)
    # ================================================================
    nav2_nodes = []
    nav2_odom = {'use_sim_time': use_sim_time}

    nav2_nodes.append(Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav2_params, nav2_odom],
    ))
    nav2_nodes.append(Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[nav2_params, nav2_odom],
    ))
    nav2_nodes.append(Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[nav2_params, nav2_odom],
    ))
    nav2_nodes.append(Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[nav2_params, nav2_odom],
    ))
    nav2_nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server', 'planner_server',
                'behavior_server', 'bt_navigator',
            ],
        }],
    ))

    delayed_nav2 = GroupAction(
        actions=[
            TimerAction(
                period=5.0,
                actions=nav2_nodes,
            ),
        ],
        condition=IfCondition(LaunchConfiguration('enable_nav2')),
    )

    # ================================================================
    # 4. RViz (optional)
    # ================================================================
    rviz_config = os.path.join(bringup_pkg, 'rviz', 'lidar3d_livox_nav.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription(
        declared_arguments + [
            ekf_node,
            cmd_vel_relay,
            delayed_nav2,
            rviz_node,
        ]
    )
