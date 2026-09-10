"""Shared front-camera bringup for live diagnostics, SLAM, and SVO replay."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    share = Path(get_package_share_directory('vision_bringup'))
    svo = LaunchConfiguration('svo_path').perform(context)
    replay = svo not in ('', 'live')
    if replay:
        recording = Path(svo).expanduser()
        if not recording.is_absolute() or not recording.is_file():
            raise ValueError('svo_path must be an existing absolute SVO/SVO2 file path')
        svo = str(recording.resolve())

    xacro_command = [
        FindExecutable(name='xacro'), ' ', str(share / 'urdf' / 'robosub_zed.urdf.xacro')
    ]
    for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        name = 'camera_to_base_' + axis
        xacro_command.extend([' ' + name + ':=', LaunchConfiguration(name)])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory('zed_wrapper')) / 'launch' /
                'zed_camera.launch.py'
            )),
            launch_arguments={
                'camera_name': 'zed_front',
                'camera_model': 'zed2i',
                'node_name': 'zed_node',
                'serial_number': LaunchConfiguration('serial_number'),
                'ros_params_override_path': LaunchConfiguration('zed_config'),
                'publish_tf': 'true',
                'publish_map_tf': LaunchConfiguration('publish_map_tf'),
                'publish_urdf': 'false',
                'svo_path': svo if replay else 'live',
                'publish_svo_clock': str(replay).lower(),
                # The clock producer must not wait for its own /clock.
                'use_sim_time': 'false',
            }.items(),
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(xacro_command), value_type=str),
                'use_sim_time': replay,
            }],
        ),
        Node(
            package='vision_bringup', executable='fused_map_exporter',
            name='fused_map_exporter', output='screen',
            condition=IfCondition(LaunchConfiguration('export_fused_map')),
            parameters=[{
                'input_topic': '/zed_front/zed_node/mapping/fused_cloud',
                'output_path': LaunchConfiguration('map_output'),
                'auto_save': True,
                'snapshot_period_sec': 5.0,
                'use_sim_time': replay,
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='vision_rviz', output='screen',
            condition=IfCondition(LaunchConfiguration('start_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': replay}],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('vision_bringup'))
    arguments = [
        DeclareLaunchArgument('serial_number', default_value='36534008'),
        DeclareLaunchArgument('svo_path', default_value='live'),
        DeclareLaunchArgument('zed_config', default_value=str(share / 'config/zed_front.yaml')),
        DeclareLaunchArgument('publish_map_tf', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=str(share / 'rviz/front_debug.rviz')),
        DeclareLaunchArgument('export_fused_map', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('map_output', default_value='maps/spatial_mapping/front_room.ply'),
    ]
    arguments.extend(
        DeclareLaunchArgument('camera_to_base_' + axis, default_value='0.0')
        for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
    )
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
