#!/usr/bin/env python3
"""Optional MuJoCo + shared controllers. Run in a ROS domain separate from hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def source(name):
        return PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ieir_simulation'), 'launch', name]))
    return LaunchDescription([
        DeclareLaunchArgument('controller_type', default_value='joint_position',
                              choices=['gravity_compensation', 'joint_position', 'cartesian_position']),
        SetEnvironmentVariable('IEIR_MUJOCO_CONFIG', 'simulate.yaml'),
        SetEnvironmentVariable('IEIR_MUJOCO_PANEL_MODE', ''),
        IncludeLaunchDescription(source('mujoco_sim.launch.py')),
        TimerAction(period=2.0, actions=[IncludeLaunchDescription(
            source('controllers.launch.py'), launch_arguments={
                'controller_type': LaunchConfiguration('controller_type')}.items())]),
    ])
