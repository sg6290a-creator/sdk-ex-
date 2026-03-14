from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_launch = PathJoinSubstitution([FindPackageShare('arm_sdk'), 'launch', 'arm.launch.py'])
    gripper_launch = PathJoinSubstitution([FindPackageShare('gripper_sdk'), 'launch', 'gripper.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('arm_use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('gripper_use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('gripper_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('gripper_baudrate', default_value='1000000'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_launch),
            launch_arguments={
                'namespace': 'arm',
                'controller_manager_name': '/arm/controller_manager',
                'frame_prefix': 'arm/',
                'use_mock_hardware': LaunchConfiguration('arm_use_mock_hardware'),
                'auto_home': 'false',
                'rviz': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gripper_launch),
            launch_arguments={
                'namespace': 'gripper',
                'port': LaunchConfiguration('gripper_port'),
                'baudrate': LaunchConfiguration('gripper_baudrate'),
                'use_mock_hardware': LaunchConfiguration('gripper_use_mock_hardware'),
            }.items(),
        ),
    ])
