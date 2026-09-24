#!/usr/bin/env python3
"""MuJoCo panel for mirroring and editing the real robot state.

This launch starts only the MuJoCo window. It does not start the CAN bridge,
real ros2_control, or the simulation controllers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='mirror_real',
            description='Panel mode: mirror_real or target_editor',
            choices=['mirror_real', 'target_editor'],
        ),
        DeclareLaunchArgument(
            'trajectory_duration',
            default_value='5.0',
            description='Seconds for the JointTrajectory sent by /mujoco_panel/send_target',
        ),
        SetEnvironmentVariable(
            'IEIR_MUJOCO_CONFIG',
            'mujoco_panel.yaml',
        ),
        SetEnvironmentVariable(
            'IEIR_MUJOCO_PANEL_MODE',
            LaunchConfiguration('mode'),
        ),
        SetEnvironmentVariable(
            'IEIR_MUJOCO_TRAJECTORY_DURATION',
            LaunchConfiguration('trajectory_duration'),
        ),
        Node(
            package='ieir_simulation',
            executable='simulate',
            name='mujoco_panel',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
        ),
    ])
