#!/usr/bin/env python3
"""
============================================================================
Dual RealSense + YOLO Launch File
============================================================================

- D405: Gripper camera (PointCloud + YOLO object detection)
- D455: Navigation camera (IMU + depth)

[사용법]
ros2 launch frjoco_bringup sensors.launch.py

============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    realsense_launch = os.path.join(
        FindPackageShare('realsense2_camera').find('realsense2_camera'),
        'launch', 'rs_launch.py',
    )

    declared_arguments = [
        DeclareLaunchArgument('d405_serial', default_value="'315122271488'"),
        DeclareLaunchArgument('d455_serial', default_value="'213622301251'"),
        DeclareLaunchArgument('enable_yolo', default_value='true'),
        DeclareLaunchArgument('yolo_model_path', default_value=''),
    ]

    # D405 — gripper camera
    d405_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            'camera_name': 'd405_cam',
            'camera_namespace': 'd405_cam',
            'serial_no': LaunchConfiguration('d405_serial'),
            'initial_reset': 'true',
            'enable_depth': 'true',
            'enable_color': 'true',
            'depth_module.depth_profile': '640x480x15',
            'depth_module.color_profile': '640x480x15',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
        }.items(),
    )

    # D455 — navigation camera (delayed 25 s to avoid USB bandwidth conflict)
    d455_camera = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch),
                launch_arguments={
                    'camera_name': 'd455',
                    'camera_namespace': 'd455',
                    'device_type': 'd455',
                    'serial_no': LaunchConfiguration('d455_serial'),
                    'depth_module.depth_profile': '640x480x15',
                    'rgb_camera.color_profile': '640x480x15',
                    'align_depth.enable': 'true',
                    'enable_gyro': 'true',
                    'enable_accel': 'true',
                    'unite_imu_method': '2',
                    'gyro_fps': '200',
                    'accel_fps': '100',
                }.items(),
            )
        ],
    )

    # Static TF bridges (URDF frame → RealSense driver frame)
    d405_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='d405_tf_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'd405', 'd405_cam_link'],
    )
    d455_tf = TimerAction(
        period=26.0,
        actions=[
            Node(
                package='tf2_ros', executable='static_transform_publisher',
                name='d455_tf_bridge',
                arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'd455_link'],
            )
        ],
    )

    # YOLO object detection (delayed 18 s for camera warmup)
    yolo_node = GroupAction(
        actions=[
            TimerAction(
                period=18.0,
                actions=[
                    Node(
                        package='yolo_realsense',
                        executable='yolo_node',
                        name='yolo_visual_publisher',
                        output='screen',
                        parameters=[{
                            'camera_frame': 'd405_optical_frame',
                            'color_topic': '/d405_cam/d405_cam/color/image_raw',
                            'depth_topic': '/d405_cam/d405_cam/aligned_depth_to_color/image_raw',
                            'camera_info_topic': '/d405_cam/d405_cam/aligned_depth_to_color/camera_info',
                            'show_visualization': False,
                            'confidence_threshold': 0.5,
                            'model_path': LaunchConfiguration('yolo_model_path'),
                        }],
                    )
                ],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('enable_yolo')),
    )

    return LaunchDescription(
        declared_arguments + [
            d405_camera,
            d405_tf,
            d455_camera,
            d455_tf,
            yolo_node,
        ]
    )
