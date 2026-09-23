#!/usr/bin/env python3
"""Start the dual-arm W3 SocketCAN bridge (can0/can1 only)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arms', default_value='dual',
                              choices=['left', 'right', 'dual'],
                              description='Open left can0, right can1, or both (default)'),
        DeclareLaunchArgument('gripper', default_value='false',
                              choices=['true', 'false'],
                              description='Include CAN ID 8 on each arm bus'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('w3_robot_bridge'), 'launch',
                'w3_robot_bridge.launch.py'])),
            launch_arguments={'gripper': LaunchConfiguration('gripper'),
                              'arms': LaunchConfiguration('arms')}.items()),
    ])
