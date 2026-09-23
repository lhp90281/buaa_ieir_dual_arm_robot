#!/usr/bin/env python3
"""Launch the UDP teleoperation bridge.

Run this after the local robot has already been brought up with
bridge.launch.py and real_robot.launch.py. The two robot hosts may use
different ROS_DOMAIN_ID values; only the UDP peer_host/ports must be reachable.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('gripper', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'role',
            description="This host's teleop role: master or slave",
            choices=['master', 'slave'],
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='no_feedback',
            description='Teleop mode: no_feedback or force_feedback',
            choices=['no_feedback', 'force_feedback'],
        ),
        DeclareLaunchArgument(
            'peer_host',
            description='Peer host IP address or hostname',
        ),
        DeclareLaunchArgument(
            'bind_host',
            default_value='0.0.0.0',
            description='Local UDP bind address',
        ),
        DeclareLaunchArgument(
            'local_port',
            default_value='15000',
            description='Local UDP receive port',
        ),
        DeclareLaunchArgument(
            'peer_port',
            default_value='15001',
            description='Peer UDP receive port',
        ),
        DeclareLaunchArgument(
            'rate_hz',
            default_value='50.0',
            description='UDP state and command update rate',
        ),
        DeclareLaunchArgument(
            'align_duration',
            default_value='5.0',
            description='Master-to-slave alignment ramp duration, seconds',
        ),
        DeclareLaunchArgument(
            'prepare_timeout',
            default_value='30.0',
            description='Seconds /teleop/prepare waits for peer UDP state',
        ),
        DeclareLaunchArgument(
            'timeout',
            default_value='0.3',
            description='Peer UDP timeout before auto-disable, seconds',
        ),
        DeclareLaunchArgument(
            'max_start_error',
            default_value='0.5',
            description='Maximum joint error allowed when enabling, rad',
        ),
        DeclareLaunchArgument(
            'max_runtime_error',
            default_value='1.0',
            description='Maximum joint error allowed while enabled, rad',
        ),
        DeclareLaunchArgument(
            'max_step',
            default_value='0.03',
            description='Maximum commanded target change per cycle, rad',
        ),
    ]

    teleop = Node(
        package='eiriarm_controllers',
        executable='teleop_joint_bridge',
        name='teleop_joint_bridge',
        output='screen',
        arguments=[
            '--gripper', LaunchConfiguration('gripper'),
            '--role', LaunchConfiguration('role'),
            '--mode', LaunchConfiguration('mode'),
            '--peer-host', LaunchConfiguration('peer_host'),
            '--bind-host', LaunchConfiguration('bind_host'),
            '--local-port', LaunchConfiguration('local_port'),
            '--peer-port', LaunchConfiguration('peer_port'),
            '--rate-hz', LaunchConfiguration('rate_hz'),
            '--align-duration', LaunchConfiguration('align_duration'),
            '--prepare-timeout', LaunchConfiguration('prepare_timeout'),
            '--timeout', LaunchConfiguration('timeout'),
            '--max-start-error', LaunchConfiguration('max_start_error'),
            '--max-runtime-error', LaunchConfiguration('max_runtime_error'),
            '--max-step', LaunchConfiguration('max_step'),
        ],
    )

    return LaunchDescription([*args, teleop])
