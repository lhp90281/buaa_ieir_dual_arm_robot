"""Single operator UI. The separately started bridge is never managed here."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace', default_value=str(Path.cwd())),
        DeclareLaunchArgument('port', default_value='8766'),
        DeclareLaunchArgument('gripper', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('preview', default_value='false', choices=['true', 'false']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ieir_bringup'), 'launch', 'web_console.launch.py'])),
            launch_arguments={
                'workspace': LaunchConfiguration('workspace'), 'port': LaunchConfiguration('port'),
                'gripper': LaunchConfiguration('gripper'), 'preview': LaunchConfiguration('preview'),
                'allow_control': 'true', 'manage_processes': 'true',
            }.items()),
    ])
