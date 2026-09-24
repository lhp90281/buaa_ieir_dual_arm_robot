#!/usr/bin/env python3
"""
Record a drag-teach trajectory of the arm(s).

Thin launch wrapper around `teach_replay record` in ieir_controllers.
Run AFTER `real_robot.launch.py` (defaults are fine -- gravity-comp mode
makes the arm hand-pushable). Stop the recording with 'q' or Ctrl-C in
the spawned terminal; the YAML file is written on exit.

NOTE: joint_position_controller MUST NOT be active during recording -- its
PD would fight the operator dragging the arm. The default
`real_robot.launch.py controller:=gravity` is correct here. If you have
joint_position_controller active, deactivate it first with:

  ros2 launch ieir_bringup go_home.launch.py duration:=0.5

(That ramp ends with gravity_compensation active again.)

Usage
-----

  # record at 50 Hz, all joints from /joint_states, until 'q' / Ctrl-C
  ros2 launch ieir_bringup record.launch.py output:=left_wave.yaml

  # record only the left arm
  ros2 launch ieir_bringup record.launch.py \
       output:=left_wave.yaml \
       joints:='left_joint_0 left_joint_1 left_joint_2 left_joint_3 \
                left_joint_4 left_joint_5 left_joint_6'

Arguments
---------

  output         REQUIRED. YAML output path.
  rate           Sample rate in Hz (default 50).
  joints         Optional space-separated joint subset (default: all in
                 /joint_states).
  max_duration   Recording length cap in seconds (default 120).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nodes(context, *args, **kwargs):
    output = LaunchConfiguration('output').perform(context).strip()
    rate = LaunchConfiguration('rate').perform(context).strip()
    max_dur = LaunchConfiguration('max_duration').perform(context).strip()
    joints = LaunchConfiguration('joints').perform(context).strip()

    if not output:
        # Match the script's argparse error path -- launch will print this
        # in red and exit non-zero rather than starting an unusable node.
        raise RuntimeError(
            "record.launch.py: 'output' is required, e.g. "
            "output:=recordings/left_wave.yaml")

    argv = [
        'record',
        '--output', output,
        '--rate', rate,
        '--max-duration', max_dur,
    ]
    if joints:
        argv.append('--joints')
        argv.extend(joints.split())

    return [Node(
        package='ieir_controllers',
        executable='teach_replay',
        name='teach_recorder',
        output='screen',
        arguments=argv,
        emulate_tty=True,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output',
            default_value='',
            description='REQUIRED. YAML output path for the recording.',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='50.0',
            description='Sample rate in Hz',
        ),
        DeclareLaunchArgument(
            'joints',
            default_value='',
            description=(
                'Optional space-separated joint subset to record. Empty '
                '(default) means: every joint published on /joint_states.'
            ),
        ),
        DeclareLaunchArgument(
            'max_duration',
            default_value='120.0',
            description='Cap on recording length in seconds',
        ),
        OpaqueFunction(function=_build_nodes),
    ])
