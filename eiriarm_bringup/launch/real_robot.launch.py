#!/usr/bin/env python3
"""
Control-side launch for the EiriArm dual 7-DoF robot on REAL hardware.

This launch starts ONLY the ros2_control / controllers / gripper side of
the stack. The W3 SocketCAN bridge is
intentionally NOT included here; it lives in `bridge.launch.py` and is
meant to run in its own terminal so its serial-IO chatter does not drown
out the controller logs.

Two-terminal workflow:

  # terminal 1 -- CAN bridge
  ros2 launch eiriarm_bringup bridge.launch.py

  # terminal 2 -- controllers (this file)
  ros2 launch eiriarm_bringup real_robot.launch.py

All knobs are exposed as launch arguments; you should never need to edit
this file or run `ros2 control switch_controllers` by hand.

Common invocations
------------------

  # Both arms, gravity-comp / teach mode, without grippers (defaults):
  ros2 launch eiriarm_bringup real_robot.launch.py

  # Both arms with joint-space PD tracking active immediately:
  ros2 launch eiriarm_bringup real_robot.launch.py controller:=joint_position

  # Joint-space PD tracking with the MuJoCo real-robot panel:
  ros2 launch eiriarm_bringup real_robot.launch.py \
       controller:=joint_position use_gui:=true

  # Gravity-comp teach mode with the MuJoCo mirror/control panel:
  ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=true

  # Both arms with the cartesian PD coordinator:
  ros2 launch eiriarm_bringup real_robot.launch.py controller:=cartesian_position

  # Single-arm bring-up (e.g. left only -- ch1):
  ros2 launch eiriarm_bringup real_robot.launch.py arms:=left

  # No gripper (e.g. swapped end-effector):
  ros2 launch eiriarm_bringup real_robot.launch.py gripper:=false

  # Custom calibration file:
  ros2 launch eiriarm_bringup real_robot.launch.py \
       offsets_yaml:=joint_offsets_dual.yaml

Argument summary
----------------

  arms           left | right | dual    (which arm(s) to expose).
  controller     gravity | joint_position | cartesian_position
                 (cartesian_position requires arms=dual).
  gripper        true | false   (start gripper_controller_node).
  offsets_yaml   path to the 14-joint zero/sign calibration YAML.
                 Relative paths are resolved from the launch working directory.
  friction_model_yaml
                 path to the friction model YAML. Relative paths are resolved
                 from the launch working directory.
  teleop_role    slave | master. Selects joint-position kp/kd profile.
                 Mainly kept for experimental force-feedback tuning.
  teleop         true | false   (start UDP teleoperation bridge).
  teleop_node_role
                 slave | master. Selects this host's teleop role.
  teleop_peer_host
                 peer IP/hostname for UDP teleoperation.
  use_gui        true | false   (start MuJoCo mirror/target-editor panel).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'arms',
            default_value='dual',
            description="Which arm(s) to expose to ros2_control: 'left', 'right' or 'dual'",
            choices=['left', 'right', 'dual'],
        ),
        DeclareLaunchArgument(
            'controller',
            default_value='gravity',
            description=(
                'Top-tier controller to spawn ACTIVE alongside the always-on '
                'gravity_compensation_controller. No manual '
                '`ros2 control switch_controllers` is required.'
            ),
            choices=['gravity', 'joint_position', 'cartesian_position'],
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value='false',
            description='Grippers installed: include their mass/model and start their controller',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'offsets_yaml',
            default_value='src/ros2_ws_config/joint_offsets_dual.yaml',
            description=(
                'Path to the 14-joint zero/sign calibration YAML. Relative '
                'paths are resolved from the launch working directory.'
            ),
        ),
        DeclareLaunchArgument(
            'friction_model_yaml',
            default_value='src/ros2_ws_config/friction_model.yaml',
            description=(
                'Path to the friction model YAML. Relative paths are '
                'resolved from the launch working directory.'
            ),
        ),
        DeclareLaunchArgument(
            'teleop_role',
            default_value='slave',
            description=(
                'Joint-position gain profile kept mainly for experimental '
                'force-feedback tuning; daily no_feedback teleop can keep '
                'the default slave profile.'
            ),
            choices=['master', 'slave'],
        ),
        DeclareLaunchArgument(
            'teleop_gains_yaml',
            default_value='',
            description=(
                'Optional YAML with master/slave joint_position kp/kd '
                'profiles. Empty uses the eiriarm_controllers default.'
            ),
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='false',
            description='Start MuJoCo mirror/target-editor panel alongside real hardware control',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Start the UDP teleoperation bridge with real_robot.launch.py',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'teleop_node_role',
            default_value='slave',
            description="This host's teleop role: master or slave",
            choices=['master', 'slave'],
        ),
        DeclareLaunchArgument(
            'teleop_mode',
            default_value='no_feedback',
            description='Teleop mode: no_feedback or experimental force_feedback',
            choices=['no_feedback', 'force_feedback'],
        ),
        DeclareLaunchArgument(
            'teleop_peer_host',
            default_value='',
            description='Peer host IP address or hostname; required when teleop:=true',
        ),
        DeclareLaunchArgument(
            'teleop_bind_host',
            default_value='0.0.0.0',
            description='Local UDP bind address for teleop',
        ),
        DeclareLaunchArgument(
            'teleop_local_port',
            default_value='15000',
            description='Local UDP receive port for teleop',
        ),
        DeclareLaunchArgument(
            'teleop_peer_port',
            default_value='15001',
            description='Peer UDP receive port for teleop',
        ),
        DeclareLaunchArgument(
            'teleop_rate_hz',
            default_value='50.0',
            description='Teleop UDP state and command update rate',
        ),
        DeclareLaunchArgument(
            'teleop_align_duration',
            default_value='5.0',
            description='Master-to-slave teleop alignment ramp duration, seconds',
        ),
        DeclareLaunchArgument(
            'teleop_prepare_timeout',
            default_value='30.0',
            description='Seconds /teleop/prepare waits for peer UDP state',
        ),
        DeclareLaunchArgument(
            'teleop_timeout',
            default_value='0.3',
            description='Peer UDP timeout before teleop auto-disable, seconds',
        ),
        DeclareLaunchArgument(
            'teleop_max_start_error',
            default_value='0.5',
            description='Maximum joint error allowed when enabling teleop, rad',
        ),
        DeclareLaunchArgument(
            'teleop_max_runtime_error',
            default_value='1.0',
            description='Maximum joint error allowed while teleop is enabled, rad',
        ),
        DeclareLaunchArgument(
            'teleop_max_step',
            default_value='0.03',
            description='Maximum teleop commanded target change per cycle, rad',
        ),
    ]

    # ---- ros2_control + controllers + gripper ----
    # NOTE: the W3 SocketCAN bridge is intentionally NOT included here. Run it
    # in a separate terminal via `ros2 launch eiriarm_bringup bridge.launch.py`.
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('eiriarm_controllers'),
                'launch',
                'dual_arm.launch.py',
            ]),
        ),
        launch_arguments={
            'arms':         LaunchConfiguration('arms'),
            'controller':   LaunchConfiguration('controller'),
            'gripper':      LaunchConfiguration('gripper'),
            'offsets_yaml': LaunchConfiguration('offsets_yaml'),
            'friction_model_yaml': LaunchConfiguration('friction_model_yaml'),
            'teleop_role': LaunchConfiguration('teleop_role'),
            'teleop_gains_yaml': LaunchConfiguration('teleop_gains_yaml'),
        }.items(),
    )

    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('eiriarm_bringup'),
                'launch',
                'mujoco_panel.launch.py',
            ]),
        ),
        launch_arguments={
            'mode': 'mirror_real',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('eiriarm_bringup'),
                'launch',
                'teleop.launch.py',
            ]),
        ),
        launch_arguments={
            'role': LaunchConfiguration('teleop_node_role'),
            'gripper': LaunchConfiguration('gripper'),
            'mode': LaunchConfiguration('teleop_mode'),
            'peer_host': LaunchConfiguration('teleop_peer_host'),
            'bind_host': LaunchConfiguration('teleop_bind_host'),
            'local_port': LaunchConfiguration('teleop_local_port'),
            'peer_port': LaunchConfiguration('teleop_peer_port'),
            'rate_hz': LaunchConfiguration('teleop_rate_hz'),
            'align_duration': LaunchConfiguration('teleop_align_duration'),
            'prepare_timeout': LaunchConfiguration('teleop_prepare_timeout'),
            'timeout': LaunchConfiguration('teleop_timeout'),
            'max_start_error': LaunchConfiguration('teleop_max_start_error'),
            'max_runtime_error': LaunchConfiguration('teleop_max_runtime_error'),
            'max_step': LaunchConfiguration('teleop_max_step'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('teleop')),
    )

    return LaunchDescription([*args, control_launch, gui_launch, teleop_launch])
