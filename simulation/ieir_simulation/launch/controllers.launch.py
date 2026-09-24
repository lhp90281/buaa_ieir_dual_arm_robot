#!/usr/bin/env python3
"""Simulation controllers: only topic hardware, never a W3/CAN gripper node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    mode = LaunchConfiguration('controller_type').perform(context)
    package = FindPackageShare('ieir_simulation')
    description = {'robot_description': ParameterValue(Command([
        'xacro ', PathJoinSubstitution([package, 'config', 'dual_arm_sim_ros2_control.urdf.xacro'])
    ]), value_type=str)}
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[description], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[description, PathJoinSubstitution([
                 package, 'config', 'dual_arm_sim_controllers.yaml'])], output='screen'),
    ]
    active = {'joint_position': 'joint_position_controller',
              'cartesian_position': 'cartesian_position_controller'}.get(mode)
    for name in ('joint_state_broadcaster', 'gravity_compensation_controller',
                 'joint_position_controller', 'cartesian_position_controller'):
        args = [name, '-c', '/controller_manager']
        if name in ('joint_position_controller', 'cartesian_position_controller') and name != active:
            args.append('--inactive')
        nodes.append(Node(package='controller_manager', executable='spawner',
                          arguments=args, output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller_type', default_value='joint_position',
                              choices=['gravity_compensation', 'joint_position', 'cartesian_position']),
        OpaqueFunction(function=launch_setup),
    ])
