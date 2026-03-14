from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_sdk')
    urdf_path = os.path.join(pkg_share, 'urdf', 'arm_sdk.urdf.xacro')
    controller_config = os.path.join(pkg_share, 'config', 'arm_controllers.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'arm_sdk.rviz')

    declared_arguments = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('controller_manager_name', default_value='/controller_manager'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('arm_backend', default_value='myactuator'),
        DeclareLaunchArgument('can_channel', default_value='0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('motor1_id', default_value='1'),
        DeclareLaunchArgument('motor2_id', default_value='2'),
        DeclareLaunchArgument('motor3_id', default_value='3'),
        DeclareLaunchArgument('motor4_id', default_value='4'),
        DeclareLaunchArgument('auto_home', default_value='true'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    robot_description_content = Command([
        'xacro ', urdf_path,
        ' arm_backend:=', LaunchConfiguration('arm_backend'),
        ' can_channel:=', LaunchConfiguration('can_channel'),
        ' baudrate:=', LaunchConfiguration('baudrate'),
        ' motor1_id:=', LaunchConfiguration('motor1_id'),
        ' motor2_id:=', LaunchConfiguration('motor2_id'),
        ' motor3_id:=', LaunchConfiguration('motor3_id'),
        ' motor4_id:=', LaunchConfiguration('motor4_id'),
        ' auto_home:=', LaunchConfiguration('auto_home'),
        ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'frame_prefix': LaunchConfiguration('frame_prefix')},
        ],
        output='screen',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[robot_description, controller_config],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('namespace'),
        arguments=['joint_state_broadcaster', '-c', LaunchConfiguration('controller_manager_name')],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=LaunchConfiguration('namespace'),
        arguments=['arm_controller', '-c', LaunchConfiguration('controller_manager_name')],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=LaunchConfiguration('namespace'),
        arguments=['-d', rviz_config],
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    )

    delay_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_rviz = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            ros2_control_node,
            TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
            delay_arm,
            delay_rviz,
        ]
    )
