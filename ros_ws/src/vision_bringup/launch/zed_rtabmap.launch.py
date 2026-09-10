"""Room mapping and localization using ZED RGB-D and ZED local odometry."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def prepare_database_path(value, mode):
    """Resolve storage without ever creating, truncating, or deleting a database."""
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping or localization')
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise ValueError('database_path must be absolute (or start with ~/)')
    path = path.resolve()
    if path.exists() and (not path.is_file() or path.stat().st_size == 0):
        raise ValueError('database_path must name a nonempty database or a new file')
    if mode == 'localization' and not path.is_file():
        raise ValueError('Localization requires an existing database; map the room first')
    path.parent.mkdir(parents=True, exist_ok=True)
    return str(path)


def launch_setup(context):
    share = Path(get_package_share_directory('vision_bringup'))
    mode = LaunchConfiguration('mode').perform(context)
    database = prepare_database_path(LaunchConfiguration('database_path').perform(context), mode)
    replay = LaunchConfiguration('svo_path').perform(context) not in ('', 'live')
    return [
        LogInfo(msg=f'RTAB-Map {mode}: {database} (existing data is preserved)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/zed_front.launch.py')),
            launch_arguments={
                'zed_config': LaunchConfiguration('zed_config'),
                'svo_path': LaunchConfiguration('svo_path'),
                'publish_map_tf': 'false',
                'export_fused_map': 'false',
                'rviz_config': str(share / 'rviz/rtabmap_slam.rviz'),
            }.items(),
        ),
        Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap',
            namespace='rtabmap', output='screen',
            parameters=[
                LaunchConfiguration('rtabmap_config'),
                {
                    'database_path': ParameterValue(database, value_type=str),
                    'use_sim_time': replay,
                    # RTAB-Map core parameters are strings, ROS parameters are typed.
                    'Mem/IncrementalMemory': ParameterValue(
                        'true' if mode == 'mapping' else 'false', value_type=str),
                    'Mem/InitWMWithAllNodes': ParameterValue('true', value_type=str),
                },
            ],
            remappings=[
                ('rgb/image', LaunchConfiguration('rgb_topic')),
                ('depth/image', LaunchConfiguration('depth_topic')),
                ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                ('odom', LaunchConfiguration('odom_topic')),
            ],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('vision_bringup'))
    topics = {
        'rgb_topic': '/zed_front/zed_node/rgb/color/rect/image',
        'depth_topic': '/zed_front/zed_node/depth/depth_registered',
        'camera_info_topic': '/zed_front/zed_node/rgb/color/rect/camera_info',
        'odom_topic': '/zed_front/zed_node/odom',
    }
    arguments = [
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localization']),
        DeclareLaunchArgument('database_path', default_value=str(
            Path.home() / '.ros/robosub/maps/bedroom.db')),
        DeclareLaunchArgument('svo_path', default_value='live'),
        DeclareLaunchArgument('zed_config', default_value=str(share / 'config/zed_front_rtabmap.yaml')),
        DeclareLaunchArgument('rtabmap_config', default_value=str(share / 'config/rtabmap.yaml')),
    ]
    arguments.extend(DeclareLaunchArgument(name, default_value=topic) for name, topic in topics.items())
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
