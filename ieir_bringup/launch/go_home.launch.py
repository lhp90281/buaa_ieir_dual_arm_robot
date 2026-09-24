#!/usr/bin/env python3
"""
Ramp the arm(s) to q=0 (home pose) and restore gravity-comp.

Thin launch wrapper around the `go_home` executable in ieir_controllers.
Run this AFTER `real_robot.launch.py` is up (in a third terminal); the
controller_manager already has joint_position_controller loaded (active
or inactive depending on `controller:=`), so this helper just:

  1. ensures joint_position_controller is active, without re-activating it
     if it is already active
  2. publishes a 1-point JointTrajectory at q=0 with
     time_from_start=duration on /joint_position_command
  3. sleeps for `duration`
  4. (optional, default ON) switches back only if this helper activated
     joint_position_controller

Usage
-----

  # default: 5s ramp, restore gravity comp afterwards
  ros2 launch ieir_bringup go_home.launch.py

  # slow 8s ramp; leave joint_position_controller active so the next
  # `replay.launch.py` does not have to re-switch it in
  ros2 launch ieir_bringup go_home.launch.py duration:=8.0 \
       restore_gravity:=false

  # only zero the left arm
  ros2 launch ieir_bringup go_home.launch.py \
       joints:='left_joint_0 left_joint_1 left_joint_2 left_joint_3 \
                left_joint_4 left_joint_5 left_joint_6'

Arguments
---------

  duration         Ramp seconds from current pose to q=0 (default 5.0).
  restore_gravity  true|false. When true (default), restores the previous
                   controller state if this helper activated
                   joint_position_controller after the ramp.
                   Set false to chain straight into a replay.
  joints           Optional, space-separated joint-list override. Empty
                   (default) -> query joint_position_controller's
                   `joints` parameter.
  command_topic    Trajectory topic (default /joint_position_command).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(s: str) -> bool:
    return s.strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _build_nodes(context, *args, **kwargs):
    duration = LaunchConfiguration('duration').perform(context).strip()
    restore = LaunchConfiguration('restore_gravity').perform(context)
    joints = LaunchConfiguration('joints').perform(context).strip()
    cmd_topic = LaunchConfiguration('command_topic').perform(context).strip()

    # `go_home` exposes its config as argparse flags, not ROS parameters,
    # so we build the argv here. Pass --joints only if the user actually
    # supplied something -- the script's nargs='*' would treat an empty
    # string as a one-element list ['',] and fail to fall back to the
    # controller's own `joints` parameter.
    argv = [
        '--duration', duration,
        '--command-topic', cmd_topic,
    ]
    if joints:
        argv.append('--joints')
        argv.extend(joints.split())
    if not _truthy(restore):
        argv.append('--no-restore-gravity')

    return [Node(
        package='ieir_controllers',
        executable='go_home',
        name='go_home',
        output='screen',
        arguments=argv,
        emulate_tty=True,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'duration',
            default_value='5.0',
            description='Ramp seconds from current pose to q=0',
        ),
        DeclareLaunchArgument(
            'restore_gravity',
            default_value='true',
            description=(
                'After the ramp, switch gravity_compensation_controller '
                'back active and joint_position_controller inactive. '
                'Set false to chain straight into replay.launch.py.'
            ),
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'joints',
            default_value='',
            description=(
                'Optional space-separated joint list override. Empty '
                "(default) means: query joint_position_controller's "
                '`joints` parameter.'
            ),
        ),
        DeclareLaunchArgument(
            'command_topic',
            default_value='/joint_position_command',
            description='JointTrajectory topic published by go_home',
        ),
        OpaqueFunction(function=_build_nodes),
    ])
