#!/usr/bin/env python3
"""Complete System Launch File
Starts MuJoCo simulation and controllers together.

Usage:
  # Joint-position controller (default, MIT position/velocity/kp/kd)
  ros2 launch eiriarm_bringup system.launch.py

  # Gravity compensation controller
  ros2 launch eiriarm_bringup system.launch.py controller_type:=gravity_compensation

  # Cartesian position controller
  ros2 launch eiriarm_bringup system.launch.py controller_type:=cartesian_position

  # Enable standalone gripper controller (requires W3 bridge topics)
  ros2 launch eiriarm_bringup system.launch.py enable_gripper:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    controller_type = LaunchConfiguration('controller_type').perform(context)
    
    # All controllers now use the same torque-actuator MJCF. The simulator
    # applies the same MIT-mode equation as the real DM motor path:
    # tau = torque_ff + kp*(q_des-q) + kd*(v_des-v).
    mujoco_config = 'simulate.yaml'
    
    # Set environment variable for MuJoCo config file selection
    set_config_env = SetEnvironmentVariable('EIRIARM_MUJOCO_CONFIG', mujoco_config)
    
    # Include MuJoCo simulation
    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('eiriarm_bringup'),
                'launch',
                'mujoco_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
        }.items()
    )
    
    # Include controllers (delayed to ensure MuJoCo is ready)
    controllers_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('eiriarm_bringup'),
                        'launch',
                        'controllers.launch.py'
                    ])
                ]),
                launch_arguments={
                    'hardware': 'sim',
                    'controller_type': LaunchConfiguration('controller_type'),
                    'enable_gripper': LaunchConfiguration('enable_gripper'),
                }.items()
            )
        ]
    )
    
    return [set_config_env, mujoco_launch, controllers_launch]


def generate_launch_description():
    # Declare launch arguments
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='joint_position',
        description='Controller type: joint_position, gravity_compensation, or cartesian_position'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Enable MuJoCo GUI'
    )
    
    enable_gripper_arg = DeclareLaunchArgument(
        'enable_gripper',
        default_value='false',
        description='Enable gripper controller'
    )
    
    return LaunchDescription([
        controller_type_arg,
        gui_arg,
        enable_gripper_arg,
        OpaqueFunction(function=launch_setup),
    ])
