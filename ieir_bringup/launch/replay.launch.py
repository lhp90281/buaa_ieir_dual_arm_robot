#!/usr/bin/env python3
"""
Replay a recorded drag-teach trajectory through joint_position_controller.

Thin launch wrapper around `teach_replay replay` in ieir_controllers.
By default the underlying script auto-switches into
joint_position_controller before streaming and back to
gravity-comp teach mode on exit, so this launch is fire-and-forget after
`real_robot.launch.py`.

Usage
-----

  # default: real-time playback with a 2s ramp-in
  ros2 launch ieir_bringup replay.launch.py input:=left_wave.yaml

  # half speed with a longer ramp-in (recorded start far from current pose)
  ros2 launch ieir_bringup replay.launch.py input:=left_wave.yaml \
       time_scale:=0.5 ramp_in:=3.0

  # legacy: do not touch the controller manager
  ros2 launch ieir_bringup replay.launch.py input:=left_wave.yaml \
       auto_switch:=false

Arguments
---------

  input          REQUIRED. YAML input path written by `record.launch.py`.
  time_scale     Playback speed, 1.0=real-time, 0.5=half (default 1.0).
  ramp_in        Seconds to ramp from current pose to recorded start
                 pose (default 2.0).
  publish_rate   Setpoint streaming rate, Hz (default 50).
  command_topic  Trajectory topic (default /joint_position_command).
  auto_switch    true|false (default true). When true, the script
                 activates joint_position_controller, keeps
                 gravity_compensation_controller active, deactivates
                 cartesian_position_controller if active, then restores
                 teach mode on exit. Set false to manage controllers by
                 hand.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(s: str) -> bool:
    return s.strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _build_nodes(context, *args, **kwargs):
    inp = LaunchConfiguration('input').perform(context).strip()
    time_scale = LaunchConfiguration('time_scale').perform(context).strip()
    ramp_in = LaunchConfiguration('ramp_in').perform(context).strip()
    publish_rate = LaunchConfiguration('publish_rate').perform(context).strip()
    cmd_topic = LaunchConfiguration('command_topic').perform(context).strip()
    auto_switch = LaunchConfiguration('auto_switch').perform(context)

    if not inp:
        raise RuntimeError(
            "replay.launch.py: 'input' is required, e.g. "
            "input:=recordings/left_wave.yaml")

    argv = [
        'replay', inp,
        '--time-scale', time_scale,
        '--ramp-in', ramp_in,
        '--publish-rate', publish_rate,
        '--command-topic', cmd_topic,
    ]
    # The script's argparse default is auto_switch=True; pass the explicit
    # negation only when the user opts out, so the default in the script
    # stays the source of truth.
    if not _truthy(auto_switch):
        argv.append('--no-auto-switch')

    return [Node(
        package='ieir_controllers',
        executable='teach_replay',
        name='teach_replayer',
        output='screen',
        arguments=argv,
        emulate_tty=True,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input',
            default_value='',
            description='REQUIRED. YAML input path from record.launch.py.',
        ),
        DeclareLaunchArgument(
            'time_scale',
            default_value='1.0',
            description='Playback speed, 1.0=real-time, 0.5=half',
        ),
        DeclareLaunchArgument(
            'ramp_in',
            default_value='2.0',
            description=(
                'Seconds to ramp from current pose to recorded start '
                'pose. kp is high; do not lower this unless start pose '
                'is close to current pose.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Setpoint streaming rate in Hz',
        ),
        DeclareLaunchArgument(
            'command_topic',
            default_value='/joint_position_command',
            description='JointTrajectory topic published by replay',
        ),
        DeclareLaunchArgument(
            'auto_switch',
            default_value='true',
            description=(
                'When true (default), activate joint_position_controller '
                'and keep gravity_compensation_controller active before '
                'replay, deactivating cartesian_position_controller if '
                'needed, then restore teach mode on exit. Set false to '
                'manage controllers by hand.'
            ),
            choices=['true', 'false'],
        ),
        OpaqueFunction(function=_build_nodes),
    ])
