"""Canonical SVO replay launch for front ZED 2i SLAM validation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py']
            )
        ),
        launch_arguments={
            'camera_name': 'zed_front',
            'camera_model': 'zed2i',
            'node_name': 'zed_node',
            'svo_path': LaunchConfiguration('svo_path'),
            'ros_params_override_path': LaunchConfiguration('zed_config'),
            'publish_tf': 'true',
            'publish_map_tf': 'true',
            'publish_urdf': 'false',
            'publish_svo_clock': 'true',
        }.items(),
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'),
                ' ',
                PathJoinSubstitution(
                    [FindPackageShare('vision_bringup'), 'urdf', 'robosub_zed.urdf.xacro']
                ),
                ' camera_to_base_x:=', LaunchConfiguration('camera_to_base_x'),
                ' camera_to_base_y:=', LaunchConfiguration('camera_to_base_y'),
                ' camera_to_base_z:=', LaunchConfiguration('camera_to_base_z'),
                ' camera_to_base_roll:=', LaunchConfiguration('camera_to_base_roll'),
                ' camera_to_base_pitch:=', LaunchConfiguration('camera_to_base_pitch'),
                ' camera_to_base_yaw:=', LaunchConfiguration('camera_to_base_yaw'),
            ]
        ),
        value_type=str,
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': True}
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='vision_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        arguments=[
            '-d',
            PathJoinSubstitution(
                [FindPackageShare('vision_bringup'), 'rviz', 'front_debug.rviz']
            ),
        ],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'svo_path',
                default_value='',
                description='Absolute path to a ZED SVO or SVO2 recording.',
            ),
            DeclareLaunchArgument(
                'zed_config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('vision_bringup'), 'config', 'zed_front.yaml']
                ),
            ),
            DeclareLaunchArgument('start_rviz', default_value='true'),
            DeclareLaunchArgument('camera_to_base_x', default_value='0.0'),
            DeclareLaunchArgument('camera_to_base_y', default_value='0.0'),
            DeclareLaunchArgument('camera_to_base_z', default_value='0.0'),
            DeclareLaunchArgument('camera_to_base_roll', default_value='0.0'),
            DeclareLaunchArgument('camera_to_base_pitch', default_value='0.0'),
            DeclareLaunchArgument('camera_to_base_yaw', default_value='0.0'),
            zed,
            robot_state_publisher,
            rviz,
        ]
    )
