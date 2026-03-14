from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mobile_launch = PathJoinSubstitution([FindPackageShare('mobile_sdk'), 'launch', 'mobile.launch.py'])
    manipulator_launch = PathJoinSubstitution([FindPackageShare('frjoco_bringup'), 'launch', 'manipulator.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('mobile_use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('arm_use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('gripper_use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('gripper_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('gripper_baudrate', default_value='1000000'),
        DeclareLaunchArgument('mobile_port_front', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('mobile_port_rear', default_value='/dev/ttyUSB1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mobile_launch),
            launch_arguments={
                'namespace': 'mobile',
                'controller_manager_name': '/mobile/controller_manager',
                'frame_prefix': 'mobile/',
                'use_mock_hardware': LaunchConfiguration('mobile_use_mock_hardware'),
                'port_front': LaunchConfiguration('mobile_port_front'),
                'port_rear': LaunchConfiguration('mobile_port_rear'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manipulator_launch),
            launch_arguments={
                'arm_use_mock_hardware': LaunchConfiguration('arm_use_mock_hardware'),
                'gripper_use_mock_hardware': LaunchConfiguration('gripper_use_mock_hardware'),
                'gripper_port': LaunchConfiguration('gripper_port'),
                'gripper_baudrate': LaunchConfiguration('gripper_baudrate'),
            }.items(),
        ),
    ])
