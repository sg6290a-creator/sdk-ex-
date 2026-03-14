#!/usr/bin/env python3
"""
============================================================================
Mobile Manipulator MoveIt2 Launch File (FrJoCo SDK)
============================================================================

SDK 기반 통합 MoveIt2 런치 파일
- Arm:     arm_sdk (ros2_control SystemInterface)
- Gripper: gripper_sdk (standalone node + GripperCommand action server)
- Mobile:  mobile_sdk (ros2_control SystemInterface)
- MoveIt2 motion planning

[사용법]
ros2 launch frjoco_bringup moveit.launch.py
ros2 launch frjoco_bringup moveit.launch.py use_mock_hardware:=false

============================================================================
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, file_path)
    with open(abs_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_config_pkg = get_package_share_directory('mobile_manipulator_moveit_config')

    # ================================================================
    # Launch Arguments
    # ================================================================
    declared_arguments = [
        DeclareLaunchArgument('use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('gripper_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('gripper_baudrate', default_value='1000000'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    use_mock = LaunchConfiguration('use_mock_hardware')
    rviz_arg = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ================================================================
    # 1. Full SDK system (arm + mobile + gripper)
    # ================================================================
    full_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('frjoco_bringup'), 'launch', 'full_system.launch.py',
            ])
        ),
        launch_arguments={
            'arm_use_mock_hardware': use_mock,
            'mobile_use_mock_hardware': use_mock,
            'gripper_use_mock_hardware': use_mock,
            'gripper_port': LaunchConfiguration('gripper_port'),
            'gripper_baudrate': LaunchConfiguration('gripper_baudrate'),
        }.items(),
    )

    # ================================================================
    # 2. MoveIt2 Configuration
    # ================================================================
    srdf_path = os.path.join(moveit_config_pkg, 'config', 'mobile_manipulator.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = load_yaml('mobile_manipulator_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('mobile_manipulator_moveit_config', 'config/joint_limits.yaml')
    ompl_yaml = load_yaml('mobile_manipulator_moveit_config', 'config/ompl_planning.yaml')
    moveit_ctrl_yaml = load_yaml('mobile_manipulator_moveit_config', 'config/moveit_controllers.yaml')

    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_ctrl_yaml.get(
            'moveit_simple_controller_manager', {}),
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    ompl_pipeline = {
        'planning_plugins': ompl_yaml.get('planning_plugins', ['ompl_interface/OMPLPlanner']),
        'request_adapters': ompl_yaml.get('request_adapters', ''),
        'response_adapters': ompl_yaml.get('response_adapters', ''),
    }

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.execution_duration_monitoring': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.0,
        'trajectory_execution.wait_for_trajectory_completion_timeout': -1.0,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ================================================================
    # 3. MoveIt Move Group Node
    # ================================================================
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_pipeline,
            trajectory_execution,
            planning_scene_monitor,
            moveit_controllers,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # 4. RViz
    rviz_config = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_pipeline,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(rviz_arg),
        output='log',
    )

    # Start MoveIt after SDK system is up (delay 8s)
    delayed_move_group = TimerAction(period=8.0, actions=[move_group_node])
    delayed_rviz = TimerAction(period=10.0, actions=[rviz_node])

    return LaunchDescription(
        declared_arguments + [
            full_system,
            delayed_move_group,
            delayed_rviz,
        ]
    )
