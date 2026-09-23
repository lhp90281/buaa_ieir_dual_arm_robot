"""Dual-arm CAN FD bridge. Grippers are absent by default."""
import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bridge_config(share, gripper, arms='dual'):
    if arms not in ('left', 'right', 'dual'):
        raise ValueError(f'Unsupported arms selection: {arms}')
    share = Path(share)
    with (share / 'config/w3_robot_bridge.yaml').open() as stream:
        config = yaml.safe_load(stream)
    for index, side in enumerate(('left', 'right')):
        channel = config['channels'][index]
        if arms != 'dual' and arms != side:
            # Channel IDs come from list indices; never renumber the right arm.
            config['channels'][index] = {'iface': '', 'motor_count': 0,
                                         'motor_configs': []}
            continue
        if gripper:
            bus = channel['iface']
            channel['motor_count'] = 8
            channel['motor_configs'].append(str(
                share / f'config/motor_config/{side}_arm/{bus}_id8.yaml'))
    return config


def launch_setup(context):
    share = Path(get_package_share_directory('w3_robot_bridge'))
    gripper = LaunchConfiguration('gripper').perform(context) == 'true'
    arms = LaunchConfiguration('arms').perform(context)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml',
                                     prefix='w3_dual_arm_', delete=False) as stream:
        yaml.safe_dump(bridge_config(share, gripper, arms), stream)
        config_path = stream.name

    def cleanup(_context):
        if os.path.exists(config_path):
            os.unlink(config_path)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='w3_robot_bridge', executable='w3_robot_bridge_node',
             name='w3_robot_bridge_node', output='screen',
             parameters=[str(share / 'config/w3_params.yaml'),
                         {'config_path': config_path}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arms', default_value='dual',
                              choices=['left', 'right', 'dual'],
                              description='Only open the selected arm CAN interfaces'),
        DeclareLaunchArgument('gripper', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
