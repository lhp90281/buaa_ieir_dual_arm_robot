#!/usr/bin/env python3
"""Real W3 controllers only. Simulation lives in ieir_simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    controller = LaunchConfiguration('controller_type').perform(context)
    controller = {'gravity_compensation': 'gravity', 'impedance': 'joint_position'}.get(
        controller, controller)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ieir_controllers'), 'launch', 'dual_arm.launch.py'])),
        launch_arguments={
            'arms': LaunchConfiguration('arms'),
            'controller': controller,
            'gripper': LaunchConfiguration('enable_gripper'),
            'offsets_yaml': LaunchConfiguration('offsets_yaml'),
            'friction_model_yaml': LaunchConfiguration('friction_model_yaml'),
            'teleop_role': LaunchConfiguration('teleop_role'),
            'teleop_gains_yaml': LaunchConfiguration('teleop_gains_yaml'),
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        # Reject the historical hardware:=sim spelling instead of starting real motors.
        DeclareLaunchArgument('hardware', default_value='real', choices=['real'],
                              description='Simulation: ros2 launch ieir_simulation system.launch.py'),
        DeclareLaunchArgument('arms', default_value='dual', choices=['left', 'right', 'dual']),
        DeclareLaunchArgument('offsets_yaml', default_value='src/ros2_ws_config/joint_offsets_dual.yaml'),
        DeclareLaunchArgument('friction_model_yaml', default_value='src/ros2_ws_config/friction_model.yaml'),
        DeclareLaunchArgument('controller_type', default_value='gravity_compensation',
                              choices=['gravity_compensation', 'joint_position', 'cartesian_position', 'impedance']),
        DeclareLaunchArgument('teleop_role', default_value='slave', choices=['master', 'slave']),
        DeclareLaunchArgument('teleop_gains_yaml', default_value=''),
        DeclareLaunchArgument('enable_gripper', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
