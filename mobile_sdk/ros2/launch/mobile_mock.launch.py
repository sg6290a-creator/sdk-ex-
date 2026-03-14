#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('mobile_sdk'), 'launch', 'mobile.launch.py'])
            ]),
            launch_arguments={
                'use_mock_hardware': 'true',
                'port_front': 'mock_front',
                'port_rear': 'mock_rear',
            }.items(),
        )
    ])
