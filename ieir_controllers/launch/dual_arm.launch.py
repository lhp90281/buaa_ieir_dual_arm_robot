#!/usr/bin/env python3
"""
ros2_control launch for the dual 7-DoF arm (ieir) on real hardware.

This is the control-side launch (controller_manager + spawners + optional
gripper). The CAN bridge is a SEPARATE responsibility and must already be
running in another terminal:

  ros2 launch ieir_bringup bridge.launch.py

The bringup wrapper `real_robot.launch.py` only forwards the args below to
this file; it does not start the bridge itself. Run this launch directly only
if the bridge is already up.

(Single board, two channels: can0 = left arm, can1 = right arm. Grippers on
can0.slot7 / can1.slot7 are NOT exposed to ros2_control here -- they are owned by
the standalone gripper_controller_node.)

Brings up:
  1. robot_state_publisher (URDF processed via xacro with the ros2_control block)
  2. controller_manager / ros2_control_node loading DMHardwareInterface
  3. joint_state_broadcaster (always active)
  4. gravity_compensation_controller (always active)
  5. joint_position_controller (always LOADED; ACTIVE iff
     controller:=joint_position, otherwise inactive). Kept loaded in
     every mode so the helper launches `go_home.launch.py` and
     `replay.launch.py` can switch into it without the operator
     touching `ros2 control switch_controllers`.
  6. cartesian_position_controller (LOADED for dual-arm mode; ACTIVE iff
     controller:=cartesian_position, otherwise inactive).
  7. gripper_controller standalone node (talks to can0.slot7 / can1.slot7
     directly, auto-calibrates open->close on startup; toggle with
     gripper:=true|false).

Usage:
  # both arms in pure gravity-comp / teach mode (default):
  ros2 launch ieir_controllers dual_arm.launch.py

  # both arms with joint-space PD tracking active out of the gate:
  ros2 launch ieir_controllers dual_arm.launch.py controller:=joint_position

  # both arms with the cartesian PD coordinator:
  ros2 launch ieir_controllers dual_arm.launch.py controller:=cartesian_position

  # only the left arm (ch1, 7 joints):
  ros2 launch ieir_controllers dual_arm.launch.py arms:=left

  # custom calibration file:
  ros2 launch ieir_controllers dual_arm.launch.py \
       offsets_yaml:=joint_offsets_dual.yaml

  # don't start the gripper controller (e.g. when you've swapped grippers):
  ros2 launch ieir_controllers dual_arm.launch.py gripper:=false

Notes for single-arm mode:
  * Both arms are loaded into Pinocchio; gripper:=false removes their payloads.
    Only the <ros2_control>
    block (and the per-joint arrays in controllers.yaml) is sliced to one arm.
  * controller:=cartesian_position is rejected in single-arm mode because the
    controller is a dual-arm coordinator.
  * Single-arm calibration still uses joint_offsets_dual.yaml; the YAML may
    contain entries for the OFF arm -- the hardware interface ignores any
    joint whose name is not declared in the active ros2_control block.
"""

import os
import tempfile
import xml.etree.ElementTree as ET
from typing import Any

import yaml
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Which slice of the per-joint arrays to keep for each 'arms' value.
# joints in dual_arm_controllers.yaml are ordered: left_joint_0..6 then right_joint_0..6
_ARM_SLICE = {
    'left':  slice(0, 7),
    'right': slice(7, 14),
    'dual':  slice(0, 14),
}

# Keys inside gravity_compensation_controller/ros__parameters whose value is
# a 14-element array aligned with 'joints' and therefore must be sliced.
_GC_JOINT_ARRAY_KEYS = ('joints', 'motor_types', 'gravity_gains', 'friction_gains')

# Same idea for joint_position_controller.
_JP_JOINT_ARRAY_KEYS = ('joints', 'kp_gains', 'kd_gains')


_VALID_TELEOP_ROLES = ('master', 'slave')


def _resolve_user_path(path: str) -> str:
    """Resolve user-supplied runtime asset paths.

    Absolute paths pass through. '~' is expanded. Relative paths are resolved
    from the directory where `ros2 launch ...` was invoked, which keeps the
    common `cd ~/ros2_ws && ros2 launch ...` workflow portable across users.
    """
    path = path.strip()
    if not path:
        return ''
    path = os.path.expanduser(path)
    if os.path.isabs(path):
        return path
    return os.path.abspath(path)


def _validate_gain_profile(
    profile: dict[str, Any],
    profile_name: str,
    n_joints: int,
) -> tuple[list[float], list[float]]:
    kp = profile.get('kp_gains')
    kd = profile.get('kd_gains')
    if not isinstance(kp, list) or not isinstance(kd, list):
        raise ValueError(
            f"teleop gain profile {profile_name!r} must contain "
            "kp_gains and kd_gains lists")
    if len(kp) != n_joints or len(kd) != n_joints:
        raise ValueError(
            f"teleop gain profile {profile_name!r} has kp/kd lengths "
            f"{len(kp)}/{len(kd)}, expected {n_joints}")
    return [float(x) for x in kp], [float(x) for x in kd]


def _apply_teleop_gains(
    cfg: dict[str, Any],
    teleop_role: str,
    gains_yaml: str,
) -> None:
    """Copy the selected master/slave profile into controller params."""
    jp = cfg.get('joint_position_controller', {}).get('ros__parameters', {})
    joints = jp.get('joints', [])
    if not isinstance(joints, list) or not joints:
        raise ValueError("joint_position_controller.joints must be a non-empty list")

    with open(gains_yaml, 'r') as f:
        gains_cfg = yaml.safe_load(f) or {}

    profile_joints = gains_cfg.get('joints')
    if isinstance(profile_joints, list) and profile_joints != joints:
        raise ValueError(
            "teleop_joint_gains.yaml joints must match "
            "joint_position_controller.joints before arm slicing")

    profiles = gains_cfg.get('profiles', {})
    if not isinstance(profiles, dict) or teleop_role not in profiles:
        raise ValueError(
            f"teleop_role {teleop_role!r} has no matching profile in {gains_yaml}")

    kp, kd = _validate_gain_profile(
        profiles[teleop_role], teleop_role, len(joints))
    jp['kp_gains'] = kp
    jp['kd_gains'] = kd

    # Keep Cartesian gains aligned with joint-space gains when present. This
    # preserves the existing bumpless joint/cartesian failover behavior.
    cart = cfg.get('cartesian_position_controller', {}).get('ros__parameters', {})
    if cart.get('joints') == joints:
        cart['kp_gains'] = list(kp)
        cart['kd_gains'] = list(kd)


def _controllers_yaml(
    src_path: str,
    arms: str,
    friction_model_yaml: str,
    teleop_role: str,
    teleop_gains_yaml: str,
) -> str:
    """Read the dual-arm controllers yaml, apply runtime path overrides, and
    slice the per-joint arrays for the selected arms when needed.

    Also drops cartesian_position_controller in single-arm mode since the
    controller is a dual-arm coordinator.
    """
    if arms not in _ARM_SLICE:
        raise ValueError(
            f"arms must be 'left', 'right' or 'dual'; got {arms!r}")

    with open(src_path, 'r') as f:
        cfg: dict[str, Any] = yaml.safe_load(f)

    _apply_teleop_gains(cfg, teleop_role, teleop_gains_yaml)

    gc = cfg.get('gravity_compensation_controller', {}).get('ros__parameters', {})
    if friction_model_yaml:
        gc['friction_model_yaml'] = friction_model_yaml

    if arms != 'dual':
        sl = _ARM_SLICE[arms]

        # Slice gravity_compensation_controller per-joint arrays.
        for key in _GC_JOINT_ARRAY_KEYS:
            val = gc.get(key)
            if isinstance(val, list) and len(val) == 14:
                gc[key] = val[sl]

        # Slice joint_position_controller per-joint arrays.
        jp = cfg.get('joint_position_controller', {}).get('ros__parameters', {})
        for key in _JP_JOINT_ARRAY_KEYS:
            val = jp.get(key)
            if isinstance(val, list) and len(val) == 14:
                jp[key] = val[sl]

        # cartesian_position_controller is a dual-arm coordinator; drop it in
        # single-arm mode.
        cfg.pop('cartesian_position_controller', None)

    out_path = os.path.join(
        tempfile.gettempdir(),
        f'dual_arm_controllers_{arms}_{teleop_role}_{os.getpid()}.yaml')
    with open(out_path, 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return out_path


_VALID_CONTROLLERS = ('gravity', 'joint_position', 'cartesian_position')


def build_robot_description(xacro_path, offsets_yaml, arms, gripper):
    document = xacro.process_file(
        xacro_path, mappings={'offsets_yaml': offsets_yaml, 'arms': arms})
    root = ET.fromstring(document.toxml())
    if not gripper:
        # Remove the complete payload subtrees, preserving the arm tool frames.
        removed = {'left_gripper_gripper_base_link', 'right_gripper_gripper_base_link'}
        joints = root.findall('joint')
        while True:
            children = {j.find('child').get('link') for j in joints
                        if j.find('parent').get('link') in removed}
            if children <= removed:
                break
            removed.update(children)
        for element in list(root):
            if ((element.tag == 'link' and element.get('name') in removed)
                    or (element.tag == 'joint'
                        and element.find('child').get('link') in removed)):
                root.remove(element)
    return ET.tostring(root, encoding='unicode')


def launch_setup(context, *args, **kwargs):
    arms = LaunchConfiguration('arms').perform(context).strip().lower()
    if arms not in _ARM_SLICE:
        raise RuntimeError(
            f"arms must be 'left', 'right' or 'dual'; got {arms!r}")

    controller = LaunchConfiguration('controller').perform(context).strip().lower()
    if controller not in _VALID_CONTROLLERS:
        raise RuntimeError(
            f"controller must be one of {_VALID_CONTROLLERS}; got {controller!r}")
    if controller == 'cartesian_position' and arms != 'dual':
        raise RuntimeError(
            f"controller:=cartesian_position requires arms:=dual; got arms:={arms!r}. "
            f"The cartesian_position_controller is a dual-arm coordinator.")

    teleop_role = LaunchConfiguration('teleop_role').perform(context).strip().lower()
    if teleop_role not in _VALID_TELEOP_ROLES:
        raise RuntimeError(
            f"teleop_role must be one of {_VALID_TELEOP_ROLES}; got {teleop_role!r}")

    enable_gripper   = LaunchConfiguration('gripper').perform(context).lower() == 'true'
    offsets_yaml = _resolve_user_path(
        LaunchConfiguration('offsets_yaml').perform(context))
    friction_model_yaml = _resolve_user_path(
        LaunchConfiguration('friction_model_yaml').perform(context))

    pkg_ieir = FindPackageShare('ieir_controllers')

    xacro_path = PathJoinSubstitution(
        [pkg_ieir, 'config', 'dual_arm_ros2_control.urdf.xacro'])

    # Resolve the source yaml absolute path (FindPackageShare gives a
    # Substitution, we need the string here to read and slice the file).
    src_controllers_yaml = os.path.join(
        FindPackageShare('ieir_controllers').perform(context),
        'config', 'dual_arm_controllers.yaml')
    teleop_gains_yaml_arg = LaunchConfiguration('teleop_gains_yaml').perform(context).strip()
    teleop_gains_yaml = (
        _resolve_user_path(teleop_gains_yaml_arg)
        if teleop_gains_yaml_arg else
        os.path.join(
            FindPackageShare('ieir_controllers').perform(context),
            'config',
            'teleop_joint_gains.yaml')
    )
    controllers_yaml = _controllers_yaml(
        src_controllers_yaml,
        arms,
        friction_model_yaml,
        teleop_role,
        teleop_gains_yaml)

    robot_description = {'robot_description': ParameterValue(
        build_robot_description(xacro_path.perform(context), offsets_yaml,
                                arms, enable_gripper), value_type=str)}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers_yaml],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Gravity compensation runs underneath every supported controller (it
    # owns the effort interface; the others claim pos/vel/stiff/damp). It is
    # always active when this launch is used.
    gravity_active = LaunchConfiguration('gravity_compensation').perform(context).strip().lower() == 'true'
    gravity_args = ['gravity_compensation_controller', '-c', '/controller_manager']
    if not gravity_active:
        gravity_args.append('--inactive')
    gravity_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=gravity_args,
        output='screen',
    )

    nodes = [
        robot_state_publisher,
        ros2_control_node,
        jsb_spawner,
        gravity_spawner,
    ]

    # Top-tier controller layout:
    #   * gravity_compensation_controller: always active (above).
    #   * joint_position_controller: ALWAYS LOADED. Active when
    #       controller==joint_position, otherwise inactive. We load it
    #       inactive even in gravity / cartesian mode so the helper
    #       scripts (`go_home`, teach `replay`) can switch into it via
    #       /controller_manager/switch_controller without the operator
    #       having to load it by hand.
    #   * cartesian_position_controller: loaded in dual-arm mode. Active
    #       iff controller==cartesian_position, otherwise inactive, so the
    #       Web UI can switch into it without re-launching.
    jp_args = ['joint_position_controller', '-c', '/controller_manager']
    if controller != 'joint_position':
        jp_args.append('--inactive')
    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=jp_args,
        output='screen',
    ))

    if arms == 'dual':
        cartesian_args = ['cartesian_position_controller', '-c', '/controller_manager']
        if controller != 'cartesian_position':
            cartesian_args.append('--inactive')
        nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            arguments=cartesian_args,
            output='screen',
        ))
    # else single-arm mode: cartesian_position_controller was removed from
    # the sliced yaml because it is a dual-arm coordinator.

    # Gripper controller: standalone node OUTSIDE ros2_control. Talks directly
    # to W3 channels 0/1, slot 7,
    # auto-runs an open->close calibration on startup. Disable in single-arm
    # mode for the OFF channel by tweaking left_enabled / right_enabled in
    # gripper_controller.yaml; here we just toggle the whole node.
    if enable_gripper:
        gripper_yaml = os.path.join(
            FindPackageShare('ieir_controllers').perform(context),
            'config', 'gripper_controller.yaml')
        gripper_params = [gripper_yaml]
        if friction_model_yaml:
            gripper_params.append({'friction_model_yaml': friction_model_yaml})
        # Disable the off channel automatically in single-arm mode so we don't
        # command the other arm's gripper.
        if arms == 'left':
            gripper_params.append({'right_enabled': False})
        elif arms == 'right':
            gripper_params.append({'left_enabled': False})
        gripper_node = Node(
            package='ieir_controllers',
            executable='gripper_controller_node',
            name='gripper_controller',
            output='screen',
            parameters=gripper_params,
        )
        nodes.append(gripper_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arms',
            default_value='dual',
            description="Which arm(s) to expose to ros2_control: 'left', 'right' or 'dual'",
            choices=['left', 'right', 'dual'],
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
            'controller',
            default_value='gravity',
            description=(
                'Top-tier controller to load active alongside '
                'gravity_compensation_controller (which is always active). '
                'joint_position_controller is always LOADED (active iff '
                'this is joint_position, otherwise inactive) so the '
                'go_home / replay helpers can switch into it without '
                'manual `ros2 control` calls.'
            ),
            choices=list(_VALID_CONTROLLERS),
        ),
        DeclareLaunchArgument(
            'teleop_role',
            default_value='slave',
            description=(
                'Joint-position gain profile to load: slave keeps the '
                'current stiff tracking gains; master loads softer gains '
                'for force-feedback operator arms.'
            ),
            choices=list(_VALID_TELEOP_ROLES),
        ),
        DeclareLaunchArgument(
            'teleop_gains_yaml',
            default_value='',
            description=(
                'Optional YAML with master/slave joint_position kp/kd '
                'profiles. Empty uses ieir_controllers/config/'
                'teleop_joint_gains.yaml.'
            ),
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value='false',
            description='Grippers installed: include their mass/model and start their controller',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'gravity_compensation',
            default_value='true',
            description=(
                'Spawn gravity_compensation_controller ACTIVE (true) or only '
                'LOADED/inactive (false). Set false to run broadcast-only '
                '(no torque), e.g. to verify joint signs in RViz by hand.'
            ),
            choices=['true', 'false'],
        ),
        OpaqueFunction(function=launch_setup),
    ])
