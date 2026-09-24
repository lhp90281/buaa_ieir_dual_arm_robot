#!/usr/bin/env python3
"""
MuJoCo Simulation Launch File
Starts the MuJoCo simulator for the dual-arm robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='dual_arm',
        description='Model name for MuJoCo simulation'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Enable MuJoCo GUI'
    )

    # MuJoCo simulator node
    mujoco_sim = Node(
        package='ieir_simulation',
        executable='simulate',
        name='mujoco_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        model_arg,
        gui_arg,
        mujoco_sim,
    ])
