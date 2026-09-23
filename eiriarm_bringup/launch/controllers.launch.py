#!/usr/bin/env python3
"""Controllers launch for real hardware or MuJoCo simulation.

Usage:
  # Real hardware control side (default). Start bridge.launch.py separately.
  ros2 launch eiriarm_bringup controllers.launch.py

  # MuJoCo topic-based hardware.
  ros2 launch eiriarm_bringup controllers.launch.py hardware:=sim controller_type:=joint_position

  # Real hardware with joint-space PD active immediately.
  ros2 launch eiriarm_bringup controllers.launch.py controller_type:=joint_position
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_VALID_HARDWARE = ('real', 'sim')
_VALID_CONTROLLER_TYPES = (
    'gravity_compensation',
    'joint_position',
    'cartesian_position',
    'impedance',
)


def _normalize_controller_type(controller_type: str) -> str:
    controller_type = controller_type.strip().lower()
    if controller_type not in _VALID_CONTROLLER_TYPES:
        raise RuntimeError(
            f"controller_type must be one of {_VALID_CONTROLLER_TYPES}; "
            f"got {controller_type!r}")
    return controller_type


def _real_controller_arg(controller_type: str) -> str:
    if controller_type == 'gravity_compensation':
        return 'gravity'
    if controller_type in ('joint_position', 'impedance'):
        return 'joint_position'
    return controller_type


def _sim_top_controller(controller_type: str) -> str | None:
    if controller_type == 'gravity_compensation':
        return None
    if controller_type in ('joint_position', 'impedance'):
        return 'joint_position_controller'
    return 'cartesian_position_controller'


def _sim_launch(context, controller_type: str):
    arms = LaunchConfiguration('arms').perform(context).strip().lower()
    if arms != 'dual':
        raise RuntimeError(
            f"hardware:=sim currently supports arms:=dual only; got arms:={arms!r}")

    enable_gripper = LaunchConfiguration('enable_gripper').perform(context).lower() == 'true'
    top_controller = _sim_top_controller(controller_type)

    pkg_eiriarm = FindPackageShare('eiriarm_controllers')
    xacro_file = PathJoinSubstitution([
        pkg_eiriarm,
        'config',
        'dual_arm_sim_ros2_control.urdf.xacro',
    ])
    sim_controllers_yaml = PathJoinSubstitution([
        pkg_eiriarm,
        'config',
        'dual_arm_sim_controllers.yaml',
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str,
        )
    }

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
        parameters=[
            robot_description,
            sim_controllers_yaml,
        ],
        output='screen',
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
    )

    joint_position_spawner_args = ['joint_position_controller', '-c', '/controller_manager']
    if top_controller != 'joint_position_controller':
        joint_position_spawner_args.append('--inactive')

    nodes_to_start = [
        robot_state_publisher,
        ros2_control_node,
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gravity_compensation_controller', '-c', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=joint_position_spawner_args,
            output='screen',
        ),
    ]

    if top_controller == 'cartesian_position_controller':
        nodes_to_start.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['cartesian_position_controller', '-c', '/controller_manager'],
                output='screen',
            )
        )

    if enable_gripper:
        nodes_to_start.append(
            Node(
                package='eiriarm_controllers',
                executable='gripper_controller_node',
                name='gripper_controller',
                output='screen',
                parameters=[PathJoinSubstitution([
                    pkg_eiriarm,
                    'config',
                    'gripper_controller.yaml',
                ])],
            )
        )

    return nodes_to_start


def _real_launch(controller_type: str):
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('eiriarm_controllers'),
                    'launch',
                    'dual_arm.launch.py',
                ])
            ),
            launch_arguments={
                'arms': LaunchConfiguration('arms'),
                'controller': _real_controller_arg(controller_type),
                'gripper': LaunchConfiguration('enable_gripper'),
                'offsets_yaml': LaunchConfiguration('offsets_yaml'),
                'friction_model_yaml': LaunchConfiguration('friction_model_yaml'),
                'teleop_role': LaunchConfiguration('teleop_role'),
                'teleop_gains_yaml': LaunchConfiguration('teleop_gains_yaml'),
            }.items(),
        )
    ]


def launch_setup(context, *args, **kwargs):
    hardware = LaunchConfiguration('hardware').perform(context).strip().lower()
    if hardware not in _VALID_HARDWARE:
        raise RuntimeError(f"hardware must be one of {_VALID_HARDWARE}; got {hardware!r}")

    controller_type = _normalize_controller_type(
        LaunchConfiguration('controller_type').perform(context))

    if hardware == 'real':
        return _real_launch(controller_type)
    return _sim_launch(context, controller_type)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware',
            default_value='real',
            description='Control target: real hardware or MuJoCo simulation',
            choices=list(_VALID_HARDWARE),
        ),
        DeclareLaunchArgument(
            'arms',
            default_value='dual',
            description="Which arm(s) to expose on real hardware: 'left', 'right' or 'dual'",
            choices=['left', 'right', 'dual'],
        ),
        DeclareLaunchArgument(
            'offsets_yaml',
            default_value='src/ros2_ws_config/joint_offsets_dual.yaml',
            description=(
                'Path to the 14-joint zero/sign calibration YAML for real '
                'hardware. Relative paths are resolved from the launch '
                'working directory.'
            ),
        ),
        DeclareLaunchArgument(
            'friction_model_yaml',
            default_value='src/ros2_ws_config/friction_model.yaml',
            description=(
                'Path to the friction model YAML for real hardware. '
                'Relative paths are resolved from the launch working directory.'
            ),
        ),
        DeclareLaunchArgument(
            'controller_type',
            default_value='gravity_compensation',
            description='Controller type: gravity_compensation, joint_position, or cartesian_position',
            choices=list(_VALID_CONTROLLER_TYPES),
        ),
        DeclareLaunchArgument(
            'teleop_role',
            default_value='slave',
            description=(
                'Real-hardware joint-position gain profile: slave keeps '
                'current stiff tracking gains; master loads softer '
                'force-feedback gains.'
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
            'enable_gripper',
            default_value='false',
            description='Enable standalone gripper controller',
            choices=['true', 'false'],
        ),
        OpaqueFunction(function=launch_setup),
    ])
