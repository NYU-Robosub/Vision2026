from pathlib import Path

import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def _default_config_path(filename: str) -> str:
    """Resolve configs from the installed package first, then the repo."""
    try:
        installed_path = Path(get_package_share_directory('bringup')) / 'config' / filename
        if installed_path.is_file():
            return str(installed_path)
    except PackageNotFoundError:
        pass

    repo_root = Path(__file__).resolve().parents[4]
    return str(repo_root / 'config' / filename)


def _load_yaml_mapping(path_string: str) -> dict:
    config_path = Path(path_string)
    if not config_path.is_file():
        raise FileNotFoundError(f'Configuration file not found: {config_path}')

    with config_path.open('r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}

    if not isinstance(data, dict):
        raise RuntimeError(f'Configuration file must contain a YAML mapping: {config_path}')

    return data


def _load_front_zed_settings(config_path: str) -> dict:
    """Read the small set of launch arguments the ZED wrapper needs."""
    data = _load_yaml_mapping(config_path)
    wildcard = data.get('/**', {})
    params = wildcard.get('ros__parameters', {})
    general = params.get('general', {})
    sensors = params.get('sensors', {})

    return {
        'camera_name': str(general.get('camera_name', 'front')),
        'camera_model': str(general.get('camera_model', 'zed2')),
        'serial_number': str(general.get('serial_number', 0)),
        'publish_imu_tf': 'true' if bool(sensors.get('publish_imu_tf', True)) else 'false',
    }


def _launch_setup(context, *args, **kwargs):
    zed_config_path = LaunchConfiguration('zed_front_config').perform(context)
    localization_config_path = LaunchConfiguration('localization_front_config').perform(context)
    detector_config_path = LaunchConfiguration('detector_front_config').perform(context)

    zed_settings = _load_front_zed_settings(zed_config_path)

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('zed_wrapper')) / 'launch' / 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_name': zed_settings['camera_name'],
            'camera_model': zed_settings['camera_model'],
            'namespace': '',
            'node_name': 'zed_node',
            'serial_number': zed_settings['serial_number'],
            'ros_params_override_path': zed_config_path,
            # The localization node owns the robot TF chain for front-only mode.
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_imu_tf': zed_settings['publish_imu_tf'],
            'publish_urdf': 'true',
        }.items(),
    )

    localization_node = Node(
        package='localization',
        executable='front_pose_node',
        name='front_pose_node',
        output='screen',
        parameters=[localization_config_path],
    )

    detector_node = Node(
        condition=IfCondition(LaunchConfiguration('enable_detector')),
        package=LaunchConfiguration('detector_package'),
        executable=LaunchConfiguration('detector_executable'),
        name='front_detector',
        output='screen',
        parameters=[detector_config_path],
    )

    return [
        LogInfo(msg=f'Loading front ZED config: {zed_config_path}'),
        LogInfo(msg=f'Loading front localization config: {localization_config_path}'),
        LogInfo(msg=f'Detector enabled: {LaunchConfiguration("enable_detector").perform(context)}'),
        zed_launch,
        localization_node,
        detector_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'zed_front_config',
                default_value=TextSubstitution(text=_default_config_path('zed_front.yaml')),
                description='Path to the front ZED wrapper configuration file.',
            ),
            DeclareLaunchArgument(
                'localization_front_config',
                default_value=TextSubstitution(text=_default_config_path('localization_front.yaml')),
                description='Path to the front localization configuration file.',
            ),
            DeclareLaunchArgument(
                'detector_front_config',
                default_value=TextSubstitution(text=_default_config_path('detector_front.yaml')),
                description='Path to the front detector configuration file.',
            ),
            DeclareLaunchArgument(
                'enable_detector',
                default_value='false',
                description='Start the optional front detector node when true.',
            ),
            DeclareLaunchArgument(
                'detector_package',
                default_value='perception',
                description='ROS 2 package that provides the front detector executable.',
            ),
            DeclareLaunchArgument(
                'detector_executable',
                default_value='front_detector_node',
                description='ROS 2 executable for the optional front detector.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
