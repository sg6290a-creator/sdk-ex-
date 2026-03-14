#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_sdk')

    declared_arguments = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('controller_manager_name', default_value='/controller_manager'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('port_front', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('port_rear', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('baudrate', default_value='19200'),
        DeclareLaunchArgument('front_driver_id', default_value='1'),
        DeclareLaunchArgument('rear_driver_id', default_value='2'),
        DeclareLaunchArgument('id_mdui', default_value='184'),
        DeclareLaunchArgument('id_mdt', default_value='183'),
        DeclareLaunchArgument('wheel_radius', default_value='0.098'),
        DeclareLaunchArgument('wheel_separation', default_value='0.32'),
        DeclareLaunchArgument('wheelbase', default_value='0.44'),
        DeclareLaunchArgument('gear_ratio', default_value='15'),
        DeclareLaunchArgument('poles', default_value='10'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    urdf_file = os.path.join(pkg_share, 'urdf', 'mobile_sdk.urdf.xacro')
    controller_config = os.path.join(pkg_share, 'config', 'diff_drive_controller.yaml')

    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
            ' port_front:=', LaunchConfiguration('port_front'),
            ' port_rear:=', LaunchConfiguration('port_rear'),
            ' baudrate:=', LaunchConfiguration('baudrate'),
            ' front_driver_id:=', LaunchConfiguration('front_driver_id'),
            ' rear_driver_id:=', LaunchConfiguration('rear_driver_id'),
            ' id_mdui:=', LaunchConfiguration('id_mdui'),
            ' id_mdt:=', LaunchConfiguration('id_mdt'),
            ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
            ' wheel_separation:=', LaunchConfiguration('wheel_separation'),
            ' wheelbase:=', LaunchConfiguration('wheelbase'),
            ' gear_ratio:=', LaunchConfiguration('gear_ratio'),
            ' poles:=', LaunchConfiguration('poles'),
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'frame_prefix': LaunchConfiguration('frame_prefix')},
        ],
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[controller_config, {'robot_description': robot_description_content}],
        output='screen',
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=LaunchConfiguration('namespace'),
                arguments=['joint_state_broadcaster', '-c', LaunchConfiguration('controller_manager_name')],
                output='screen',
            )
        ]
    )

    diff_drive_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=LaunchConfiguration('namespace'),
                arguments=['diff_drive_controller', '-c', LaunchConfiguration('controller_manager_name')],
                output='screen',
            )
        ]
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
        ]
    )
