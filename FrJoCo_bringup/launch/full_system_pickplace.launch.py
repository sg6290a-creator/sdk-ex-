#!/usr/bin/env python3
"""
============================================================================
Full System with Pick & Place Launch File
============================================================================

모든 컴포넌트 통합 실행:
1. SDK HW system (arm + mobile + gripper)
2. MoveIt2 motion planning
3. EKF sensor fusion
4. Dual RealSense + YOLO
5. Web Interface
6. Pick & Place controller (topic_pick_place.py)

[사용법]
ros2 launch frjoco_bringup full_system_pickplace.launch.py
ros2 launch frjoco_bringup full_system_pickplace.launch.py use_mock_hardware:=false

============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('frjoco_bringup')

    declared_arguments = [
        DeclareLaunchArgument('use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('enable_pickplace', default_value='true'),
        DeclareLaunchArgument('enable_nav2', default_value='true'),
        DeclareLaunchArgument('enable_sensors', default_value='true'),
        DeclareLaunchArgument('enable_web', default_value='true'),
        DeclareLaunchArgument('gripper_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('gripper_baudrate', default_value='1000000'),
    ]

    use_mock = LaunchConfiguration('use_mock_hardware')
    rviz_arg = LaunchConfiguration('rviz')

    # ================================================================
    # 1. MoveIt (includes full_system SDK launch) — immediate
    # ================================================================
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'moveit.launch.py')
        ),
        launch_arguments={
            'use_mock_hardware': use_mock,
            'rviz': rviz_arg,
            'gripper_port': LaunchConfiguration('gripper_port'),
            'gripper_baudrate': LaunchConfiguration('gripper_baudrate'),
        }.items(),
    )

    # ================================================================
    # 2. Navigation (EKF + Nav2) — 3s delay
    # ================================================================
    navigation_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={
                    'enable_nav2': LaunchConfiguration('enable_nav2'),
                }.items(),
            )
        ],
    )

    # ================================================================
    # 3. Sensors (Dual RealSense + YOLO) — 5s delay
    # ================================================================
    sensors_launch = GroupAction(
        actions=[
            TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(bringup_pkg, 'launch', 'sensors.launch.py')
                        ),
                    )
                ],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('enable_sensors')),
    )

    # ================================================================
    # 4. Web Interface — 8s delay
    # ================================================================
    web_launch = GroupAction(
        actions=[
            TimerAction(
                period=8.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                FindPackageShare('robot_web_interface'),
                                'launch', 'web_interface.launch.py',
                            ])
                        ),
                    )
                ],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('enable_web')),
    )

    # ================================================================
    # 5. Pick & Place Controller — 12s delay (all systems ready)
    # ================================================================
    pick_place = GroupAction(
        actions=[
            TimerAction(
                period=12.0,
                actions=[
                    Node(
                        package='frjoco_bringup',
                        executable='topic_pick_place.py',
                        name='topic_pick_place_controller',
                        output='screen',
                        parameters=[{
                            'approach_height': 0.08,
                            'retreat_height': 0.12,
                            'gripper_open_position': 0.019,
                            'gripper_close_position': -0.01,
                            'gripper_max_effort': 50.0,
                            'camera_frame': 'd405_color_optical_frame',
                            'base_frame': 'base_link',
                            'target_frame': 'end_effector_link',
                            'planning_group': 'arm',
                            'end_effector_link': 'end_effector_link',
                            'place_x': 0.0,
                            'place_y': 0.25,
                            'place_z': 0.15,
                        }],
                    )
                ],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('enable_pickplace')),
    )

    return LaunchDescription(
        declared_arguments + [
            moveit_launch,
            navigation_launch,
            sensors_launch,
            web_launch,
            pick_place,
        ]
    )
