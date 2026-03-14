from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('gripper_sdk')
    params_file = os.path.join(pkg_share, 'config', 'gripper_params.yaml')

    ns = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='gripper'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('enable_action_server', default_value='true'),

        # --- gripper driver node ---
        Node(
            package='gripper_sdk',
            executable='gripper_node.py',
            name='gripper_node',
            namespace=ns,
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'use_mock_hardware': ParameterValue(LaunchConfiguration('use_mock_hardware'), value_type=bool),
            }],
        ),

        # --- GripperCommand action server (for MoveIt2) ---
        Node(
            package='gripper_sdk',
            executable='gripper_action_server.py',
            name='gripper_action_server',
            namespace=ns,
            output='screen',
            parameters=[params_file],
            condition=IfCondition(
                LaunchConfiguration('enable_action_server')),
        ),
    ])
