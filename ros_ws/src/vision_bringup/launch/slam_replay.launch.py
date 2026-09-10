"""Compatibility entry point for ZED-only SVO diagnostics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('svo_path', description='Absolute path to an existing SVO/SVO2.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('vision_bringup'), 'launch', 'zed_front.launch.py'
        ]))),
    ])
