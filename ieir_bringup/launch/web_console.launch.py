"""Attach by default; optional operator-controlled process management, never bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix
from pathlib import Path


def start(context):
    executable = Path(get_package_prefix('ieir_bringup')) / 'lib/ieir_bringup/web_console'
    command = [str(executable), '--workspace', LaunchConfiguration('workspace').perform(context),
               '--port', LaunchConfiguration('port').perform(context)]
    for parameter, flag in [('preview', '--preview'), ('allow_control', '--allow-control'),
                            ('gripper', '--gripper'), ('manage_processes', '--manage-processes')]:
        if LaunchConfiguration(parameter).perform(context) == 'true':
            command.append(flag)
    return [ExecuteProcess(cmd=command, output='screen', sigterm_timeout='60', sigkill_timeout='10')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace', default_value=str(Path.cwd()), description='Workspace containing src/description and src/ros2_ws_config'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('preview', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('allow_control', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('manage_processes', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('gripper', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start),
    ])
