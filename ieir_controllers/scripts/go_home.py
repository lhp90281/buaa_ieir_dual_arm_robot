#!/usr/bin/env python3
"""go_home.py -- ramp the arm(s) to q=0 over N seconds, then restore gravity comp.

Sequence:
    1. Read the position controller's `joints` parameter so we know which
       joints to drive (and at what order).
    2. Ensure joint_position_controller is active while keeping
       gravity_compensation_controller active; deactivate cartesian only if
       it is currently active. If the position controller was already
       active, no switch request is sent for it.
    3. Publish a 1-point JointTrajectory with positions all zero and
       time_from_start = --duration. The controller's pre-roll segment
       linearly interpolates from hold_pos_ to zero over that window.
    4. Sleep --duration (+ small margin) so the ramp completes.
    5. If requested, deactivate joint_position_controller only if this
       script activated it; gravity compensation remains active throughout.

Cleanup guarantees:
    - On Ctrl-C / exception, step 5 still runs (try/finally), so a controller
      state changed by this script is restored.
    - --no-restore-gravity skips step 5 if you want to immediately replay
      another trajectory after the ramp without re-switching.

Examples:
    # default: 5s ramp, restore gravity comp
    ros2 run ieir_controllers go_home

    # slow 8s ramp; leave joint_position_controller active for the next
    # ros2 topic pub / teach_replay you intend to run
    ros2 run ieir_controllers go_home --duration 8.0 --no-restore-gravity

    # only zero the left arm
    ros2 run ieir_controllers go_home --joints \
        left_joint_0 left_joint_1 left_joint_2 left_joint_3 \
        left_joint_4 left_joint_5 left_joint_6
"""
import argparse
import signal
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions

from controller_manager_msgs.srv import SwitchController, ListControllers
from rcl_interfaces.srv import GetParameters
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ParameterType.PARAMETER_STRING_ARRAY -- hard-coded so we don't need a
# runtime import of rcl_interfaces.msg.ParameterType just for one constant.
PARAM_STRING_ARRAY = 9


def _call_service(node, client, request, timeout=5.0):
    """Wait for the service, call it, spin until the future completes."""
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    fut = client.call_async(request)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


def query_controller_joints(node, controller_name):
    """Return the 'joints' string-array parameter of `controller_name`."""
    srv = f'/{controller_name}/get_parameters'
    client = node.create_client(GetParameters, srv)
    req = GetParameters.Request()
    req.names = ['joints']
    resp = _call_service(node, client, req, timeout=3.0)
    node.destroy_client(client)
    if resp is None or not resp.values:
        return None
    v = resp.values[0]
    if v.type != PARAM_STRING_ARRAY:
        node.get_logger().warn(
            f"/{controller_name}/joints has unexpected type {v.type}")
        return None
    return list(v.string_array_value)


def controller_is_active(node, controller_name):
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            return False
        req = ListControllers.Request()
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        resp = fut.result()
        if resp is None:
            return False
        for c in resp.controller:
            if c.name == controller_name:
                return c.state == 'active'
        return False
    finally:
        node.destroy_client(client)


def build_switch_plan(node, position_controller, gravity_controller,
                      cartesian_controller):
    """Return (activate, deactivate, position_was_active).

    The controller_manager switch service is not idempotent in STRICT mode:
    asking it to activate a controller that is already active can return
    ok=False. Build the smallest switch request for the current state.
    """
    position_was_active = controller_is_active(node, position_controller)
    activate = []
    if not position_was_active:
        activate.append(position_controller)
    if not controller_is_active(node, gravity_controller):
        activate.append(gravity_controller)

    deactivate = []
    if controller_is_active(node, cartesian_controller):
        deactivate.append(cartesian_controller)

    return activate, deactivate, position_was_active


def switch_controllers(node, activate=None, deactivate=None, strict=True):
    """Atomic activate/deactivate via /controller_manager/switch_controller.

    Prefer STRICT so we never end up in a half-switched state where gravity
    compensation is dropped before the replacement controller is active.
    """
    activate = list(activate or [])
    deactivate = list(deactivate or [])
    if not activate and not deactivate:
        return True
    client = node.create_client(
        SwitchController, '/controller_manager/switch_controller')
    req = SwitchController.Request()
    req.activate_controllers = activate
    req.deactivate_controllers = deactivate
    req.strictness = (
        SwitchController.Request.STRICT if strict
        else SwitchController.Request.BEST_EFFORT)
    req.activate_asap = False
    resp = _call_service(node, client, req, timeout=10.0)
    node.destroy_client(client)
    if resp is None:
        node.get_logger().error(
            "/controller_manager/switch_controller did not respond")
        return False
    if not resp.ok:
        node.get_logger().error(
            f"switch_controller returned ok=False "
            f"(activate={activate}, deactivate={deactivate})")
        return False
    node.get_logger().info(
        f"controllers switched (+{activate} -{deactivate})")
    return True


def build_zero_trajectory(joints, duration_sec):
    """1-point JointTrajectory: q_target = 0 reached over `duration_sec`."""
    traj = JointTrajectory()
    traj.joint_names = list(joints)
    traj.header.stamp.sec = 0
    traj.header.stamp.nanosec = 0
    pt = JointTrajectoryPoint()
    pt.positions = [0.0] * len(joints)
    sec = int(duration_sec)
    nsec = int(round((duration_sec - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    pt.time_from_start = Duration(sec=sec, nanosec=nsec)
    traj.points.append(pt)
    return traj


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--duration', type=float, default=5.0,
                   help='ramp seconds from current pose to q=0 (default 5.0)')
    p.add_argument('--position-controller', default='joint_position_controller',
                   help='controller to drive the ramp '
                        '(default joint_position_controller)')
    p.add_argument('--gravity-controller', default='gravity_compensation_controller',
                   help='controller to keep active during the ramp '
                        '(default gravity_compensation_controller)')
    p.add_argument('--cartesian-controller', default='cartesian_position_controller',
                   help='controller to deactivate if active '
                        '(default cartesian_position_controller)')
    p.add_argument('--command-topic', default='/joint_position_command',
                   help='trajectory topic (default /joint_position_command)')
    p.add_argument('--joints', nargs='*', default=None,
                   help='override joint list (default: query controller)')
    p.add_argument('--no-restore-gravity', action='store_true',
                   help='leave joint_position_controller active afterwards')
    # Strip ROS 2 CLI args (e.g. `--ros-args -r __node:=go_home` injected
    # by launch_ros.actions.Node) before argparse sees them; otherwise
    # argparse aborts with "unrecognized arguments". Same pattern is
    # used in zero_at_current_pose.py / joint_state_translator.py.
    argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = p.parse_args(argv)

    if args.duration <= 0.0:
        print("ERROR: --duration must be > 0", file=sys.stderr)
        return 2

    # Take over Ctrl-C so we can do clean cleanup (rclpy's default SIGINT
    # handler would shutdown the context, breaking our restore step).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('go_home')
    stop_flag = {'val': False}

    def on_signal(signum, frame):
        stop_flag['val'] = True

    prev_sigint = signal.signal(signal.SIGINT, on_signal)
    prev_sigterm = signal.signal(signal.SIGTERM, on_signal)

    rc = 0
    switched_to_position = False
    try:
        # ---- resolve joint list ----------------------------------------
        joints = args.joints
        if not joints:
            joints = query_controller_joints(node, args.position_controller)
            if not joints:
                node.get_logger().error(
                    f"could not read 'joints' from /{args.position_controller}; "
                    "is it loaded? Otherwise pass --joints explicitly.")
                return 1
        node.get_logger().info(
            f"target: {len(joints)} joint(s) -> 0 over {args.duration:.2f}s")
        for j in joints:
            node.get_logger().info(f"  - {j}")

        # ---- switch to position controller -----------------------------
        activate, deactivate, position_was_active = build_switch_plan(
            node,
            args.position_controller,
            args.gravity_controller,
            args.cartesian_controller)
        if not switch_controllers(
                node,
                activate=activate,
                deactivate=deactivate):
            return 1
        # Confirm the position controller is really active before streaming.
        deadline = time.time() + 5.0
        while time.time() < deadline and not stop_flag['val']:
            if controller_is_active(node, args.position_controller):
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        else:
            node.get_logger().error(
                f"{args.position_controller} did not become active; aborting before publish")
            return 1
        switched_to_position = not position_was_active

        # ---- publish q=0 trajectory ------------------------------------
        pub = node.create_publisher(
            JointTrajectory, args.command_topic,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE),
        )
        node.get_logger().info(
            f"waiting for subscriber on {args.command_topic} ...")
        deadline = time.time() + 5.0
        while (pub.get_subscription_count() == 0
               and time.time() < deadline
               and not stop_flag['val']):
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            node.get_logger().error(
                f"no subscriber on {args.command_topic} after activating "
                f"{args.position_controller}")
            return 1

        traj = build_zero_trajectory(joints, args.duration)
        pub.publish(traj)
        node.get_logger().info(
            f"published q=0 trajectory; waiting {args.duration:.2f}s ...")

        # ---- wait for ramp completion ----------------------------------
        end_t = time.time() + args.duration + 0.3
        interrupted = False
        while time.time() < end_t:
            if stop_flag['val']:
                interrupted = True
                break
            rclpy.spin_once(node, timeout_sec=0.05)

        if interrupted:
            node.get_logger().info(
                "interrupted; restoring gravity comp (arm holds wherever it is)")
            rc = 130  # conventional Ctrl-C exit code
        else:
            node.get_logger().info("ramp complete; q ~= 0")
    finally:
        # ---- restore back to teach mode (gravity stays active) ----------
        # Gravity compensation is already active in the default bringup
        # and must remain active; the only thing we need to release is the
        # joint-position controller.
        if switched_to_position and not args.no_restore_gravity:
            switch_controllers(
                node,
                activate=[],
                deactivate=[args.position_controller])
        signal.signal(signal.SIGINT, prev_sigint)
        signal.signal(signal.SIGTERM, prev_sigterm)
        rclpy.try_shutdown()

    return rc


if __name__ == '__main__':
    sys.exit(main())
