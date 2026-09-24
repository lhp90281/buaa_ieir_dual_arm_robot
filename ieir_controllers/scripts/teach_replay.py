#!/usr/bin/env python3
"""Drag-teach + replay through joint_position_controller.

Workflow:
  1. Bring up the arm (gravity_compensation_controller active,
     joint_position_controller inactive).
  2. `record` --> drag the arm by hand; samples /joint_states at a fixed
                  rate and saves the trajectory to a YAML file. Stop with
                  Ctrl-C (or the --max-duration timeout).
  3. `replay` --> by default, atomically activates
                  joint_position_controller, keeps
                  gravity_compensation_controller active, deactivates
                  cartesian_position_controller if needed, then streams
                  JointTrajectory setpoints on /joint_position_command.
                  On exit (clean or Ctrl-C), joint_position_controller is
                  deactivated if this helper activated it, leaving the arm
                  in gravity-comp teach mode again.

Examples:
    # record at 50 Hz, all joints from /joint_states, until Ctrl-C
    teach_replay record -o left_wave.yaml --rate 50

    # record only one arm
    teach_replay record -o left_wave.yaml --joints \
        left_joint_0 left_joint_1 left_joint_2 left_joint_3 \
        left_joint_4 left_joint_5 left_joint_6

    # replay at half speed with a 3-second ramp from current pose to the
    # recorded start pose (kp is high; ramp_in cushions the entry).
    teach_replay replay left_wave.yaml --time-scale 0.5 --ramp-in 3.0

    # replay without touching the controller manager (use the legacy
    # workflow where you switch controllers by hand before/after)
    teach_replay replay left_wave.yaml --no-auto-switch

Pre-flight checks:
    Record: joint_position_controller MUST be inactive, otherwise its PD
    fights the user dragging the arm. The default
    `real_robot.launch.py controller:=gravity` (or any non-
    joint_position mode) is correct. The script does NOT auto-deactivate
    it; if it is active, run:
        ros2 control switch_controllers --deactivate joint_position_controller

    Replay: joint_position_controller must be LOADED. The default
    dual_arm.launch.py loads it in every mode (active or inactive),
    so this is already the case. With auto-switch enabled (default)
    the script handles activation/deactivation for you.
"""
import argparse
import bisect
import select
import signal
import sys
import termios
import time
import tty
from pathlib import Path

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController, ListControllers


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


def _build_switch_plan(node, position_controller, gravity_controller,
                       cartesian_controller):
    """Return (activate, deactivate, position_was_active).

    STRICT switch_controller is not idempotent; do not ask it to activate a
    controller that is already active.
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


def _switch_controllers(node, activate=None, deactivate=None, timeout=10.0, strict=True):
    """Atomic activate/deactivate via /controller_manager/switch_controller.

    Prefer STRICT so we never drop gravity compensation before the
    replacement controller is active.
    """
    activate = list(activate or [])
    deactivate = list(deactivate or [])
    if not activate and not deactivate:
        return True
    client = node.create_client(
        SwitchController, '/controller_manager/switch_controller')
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            node.get_logger().error(
                '/controller_manager/switch_controller not available')
            return False
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = (
            SwitchController.Request.STRICT if strict
            else SwitchController.Request.BEST_EFFORT)
        req.activate_asap = False
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        resp = fut.result()
        if resp is None:
            node.get_logger().error(
                '/controller_manager/switch_controller did not respond')
            return False
        if not resp.ok:
            node.get_logger().error(
                f'switch_controller returned ok=False '
                f'(activate={activate}, deactivate={deactivate})')
            return False
        node.get_logger().info(
            f'controllers switched (+{activate} -{deactivate})')
        return True
    finally:
        node.destroy_client(client)


def _setup_keyboard_listener():
    """Put stdin into cbreak mode and return (restore_fn, key_pressed_fn).

    key_pressed_fn() returns the next character if one is available without
    blocking, or None. When stdin is not a TTY (e.g. piped input), the key
    poll always returns None and only Ctrl-C will break the loop.
    """
    if not sys.stdin.isatty():
        return (lambda: None), (lambda: None)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def restore():
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def key_pressed():
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    return restore, key_pressed


# =============================================================================
# record
# =============================================================================
def cmd_record(args) -> int:
    rclpy.init()
    node = rclpy.create_node('teach_recorder')

    latest = {'msg': None}

    def on_state(msg: JointState):
        latest['msg'] = msg

    node.create_subscription(
        JointState, '/joint_states', on_state, qos_profile_sensor_data)

    # Wait for first /joint_states.
    node.get_logger().info('Waiting for /joint_states ...')
    deadline = time.time() + 5.0
    while latest['msg'] is None and time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    if latest['msg'] is None:
        node.get_logger().error(
            "No /joint_states received in 5s. Is the controller_manager up?")
        rclpy.try_shutdown()
        return 1

    # Resolve joint list (filter to --joints if given).
    msg0 = latest['msg']
    available = list(msg0.name)
    if args.joints:
        joints = [j for j in args.joints if j in available]
        missing = [j for j in args.joints if j not in available]
        if missing:
            node.get_logger().warn(
                f"Joints not present in /joint_states: {missing}")
        if not joints:
            node.get_logger().error(
                f"None of --joints are in /joint_states {available}")
            rclpy.try_shutdown()
            return 1
    else:
        joints = available

    period = 1.0 / args.rate
    node.get_logger().info(
        f"Recording {len(joints)} joint(s) at {args.rate:.1f} Hz "
        f"(max {args.max_duration:.0f}s). Drag the arm now. "
        f"Press 'q' or Ctrl-C to stop.")
    node.get_logger().info(f"  joints: {joints}")

    # Put stdin into cbreak mode so we can poll for 'q' without blocking.
    # Keep restore_kb in finally so the terminal is sane on any exit path
    # (KeyboardInterrupt, exception, normal break).
    restore_kb, key_pressed = _setup_keyboard_listener()

    samples = []  # list of (t_rel, [positions])
    stop_reason = 'unknown'
    t_start = time.time()
    next_tick = t_start
    last_log = t_start
    try:
        while rclpy.ok():
            ch = key_pressed()
            if ch in ('q', 'Q'):
                stop_reason = f"key '{ch}' pressed"
                break

            now = time.time()
            elapsed = now - t_start
            if elapsed >= args.max_duration:
                stop_reason = f"max_duration ({args.max_duration:.1f}s) reached"
                break
            # Sleep until next sample tick (also dispatches subscriber callbacks).
            if now < next_tick:
                rclpy.spin_once(node, timeout_sec=min(period, next_tick - now))
                continue
            # Catch up: if we fell behind several ticks, drop them and resume
            # at the current local time so duration stays meaningful.
            while next_tick <= now:
                next_tick += period

            msg = latest['msg']
            name_to_pos = dict(zip(msg.name, msg.position))
            try:
                positions = [name_to_pos[j] for j in joints]
            except KeyError as e:
                node.get_logger().warn(
                    f"Joint {e} dropped from /joint_states; sample skipped")
                continue
            samples.append((elapsed, positions))

            # Throttle progress log to ~1 Hz.
            if now - last_log >= 1.0:
                last_log = now
                node.get_logger().info(
                    f"  {len(samples)} samples / {elapsed:.1f}s")
    except KeyboardInterrupt:
        stop_reason = 'Ctrl-C'
    finally:
        # Restore terminal first so any subsequent print/log behaves
        # normally even if rclpy or the YAML write below misbehaves.
        restore_kb()
        # try_shutdown is a no-op if rclpy already shut down (which is
        # what rclpy's default SIGINT handler does on Ctrl-C). The plain
        # rclpy.shutdown() would raise RCLError in that case and abort
        # the function before the YAML save below runs.
        rclpy.try_shutdown()

    print(f"stopped: {stop_reason}; captured {len(samples)} samples")

    if len(samples) < 2:
        print(f"ERROR: only {len(samples)} sample(s) recorded; "
              "nothing useful to save", file=sys.stderr)
        return 1

    # ---- write YAML --------------------------------------------------------
    # Compact one-line-per-sample format: easier to diff/inspect than the
    # default block style, still valid YAML loadable by yaml.safe_load.
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    duration = samples[-1][0]
    with out_path.open('w') as f:
        yaml.safe_dump({
            'joint_names': joints,
            'rate_hz': float(args.rate),
            'duration_sec': float(duration),
            'num_samples': len(samples),
        }, f, sort_keys=False)
        f.write('samples:\n')
        for t, q in samples:
            q_str = '[' + ', '.join(f'{x:.6f}' for x in q) + ']'
            f.write(f"- {{t: {t:.4f}, q: {q_str}}}\n")
    print(f"Saved {len(samples)} samples ({duration:.2f}s) to {out_path}")
    return 0


# =============================================================================
# replay
# =============================================================================
def cmd_replay(args) -> int:
    in_path = Path(args.input)
    if not in_path.is_file():
        print(f"ERROR: {in_path} not found", file=sys.stderr)
        return 1
    with in_path.open() as f:
        data = yaml.safe_load(f)

    joints = list(data.get('joint_names') or [])
    raw_samples = list(data.get('samples') or [])
    if len(joints) == 0 or len(raw_samples) < 2:
        print(f"ERROR: invalid trajectory in {in_path}", file=sys.stderr)
        return 1

    # Normalise samples and sort by t (defensive; record() already writes
    # them in order). bisect-based interpolation below assumes sorted t.
    samples = []
    for s in raw_samples:
        t = float(s['t'])
        q = list(s['q'])
        if len(q) != len(joints):
            print(f"ERROR: sample at t={t:.3f} has {len(q)} positions "
                  f"but joint_names has {len(joints)}", file=sys.stderr)
            return 1
        samples.append((t, q))
    samples.sort(key=lambda x: x[0])
    ts = [s[0] for s in samples]
    qs = [s[1] for s in samples]
    n_dof = len(joints)

    scale = max(1e-3, args.time_scale)
    ramp_in = max(0.0, args.ramp_in)
    pub_rate = max(5.0, args.publish_rate)
    pub_period = 1.0 / pub_rate
    # time_from_start of every streamed point: how long the controller has
    # to interpolate from its current setpoint to the new one. Keep it >=
    # 2 * publish_period so the pre-roll never finishes before the next
    # update arrives (which would let the joint snap to the new target).
    tfs = max(2.0 * pub_period, 0.04)
    t_first = ts[0]
    t_last = ts[-1]

    # ---- rclpy init with NO built-in signal handlers --------------------
    # rclpy's default SIGINT handler shuts down the context immediately,
    # which makes the next publish fail with "publisher's context is
    # invalid". We install our own so Ctrl-C just sets a flag; cleanup
    # (final hold publish + try_shutdown) runs in the main loop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('teach_replayer')
    stop_flag = {'val': False}

    def on_signal(signum, frame):
        stop_flag['val'] = True

    prev_sigint = signal.signal(signal.SIGINT, on_signal)
    prev_sigterm = signal.signal(signal.SIGTERM, on_signal)

    # ---- auto-switch INTO joint_position_controller ---------------------
    # Keep gravity compensation alive; only kick out cartesian if it owns
    # the position interfaces. We confirm the joint_position controller is
    # active before streaming any trajectory.
    switched_in = False
    if args.auto_switch:
        activate, deactivate, position_was_active = _build_switch_plan(
            node,
            args.position_controller,
            args.gravity_controller,
            args.cartesian_controller)
        if not _switch_controllers(
                node,
                activate=activate,
                deactivate=deactivate):
            node.get_logger().error(
                'auto-switch failed; aborting replay. Pass '
                '--no-auto-switch to run the legacy manual workflow.')
            rclpy.try_shutdown()
            signal.signal(signal.SIGINT, prev_sigint)
            signal.signal(signal.SIGTERM, prev_sigterm)
            return 1
        switched_in = not position_was_active

    def _restore_switch():
        # Called from every exit path below. Safe to call even when we
        # never switched (no-op). The node is still alive here because
        # rclpy.try_shutdown() runs AFTER us in the same finally block.
        # Gravity compensation is already active in the default bringup,
        # so restoring teach mode only needs to release the position controller.
        if switched_in:
            _switch_controllers(
                node,
                activate=[],
                deactivate=[args.position_controller])

    pub = node.create_publisher(
        JointTrajectory, args.command_topic,
        QoSProfile(depth=1,
                   reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.VOLATILE),
    )

    # ---- wait for the controller's subscription to attach ---------------
    node.get_logger().info(
        f"Waiting for subscriber on {args.command_topic} ...")
    deadline = time.time() + 5.0
    while (pub.get_subscription_count() == 0
           and time.time() < deadline
           and not stop_flag['val']):
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().error(
            f"No subscriber on {args.command_topic}. Is "
            f"joint_position_controller active?")
        _restore_switch()
        rclpy.try_shutdown()
        signal.signal(signal.SIGINT, prev_sigint)
        signal.signal(signal.SIGTERM, prev_sigterm)
        return 1

    if args.auto_switch:
        deadline = time.time() + 5.0
        while time.time() < deadline and not stop_flag['val']:
            if pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        if pub.get_subscription_count() == 0:
            node.get_logger().error(
                f'No subscriber on {args.command_topic} after switch; aborting replay')
            _restore_switch()
            rclpy.try_shutdown()
            signal.signal(signal.SIGINT, prev_sigint)
            signal.signal(signal.SIGTERM, prev_sigterm)
            return 1

    q_first = ', '.join(f'{x:+.3f}' for x in qs[0])
    q_last = ', '.join(f'{x:+.3f}' for x in qs[-1])
    total = (t_last - t_first) / scale + ramp_in
    node.get_logger().info(
        f"Loaded {len(samples)} samples, {n_dof} joint(s); "
        f"streaming at {pub_rate:.0f} Hz, total ~{total:.2f}s "
        f"(time_scale={scale}, ramp_in={ramp_in}s)")
    node.get_logger().info(f"  first: [{q_first}]")
    node.get_logger().info(f"  last:  [{q_last}]")
    node.get_logger().info(
        "controls: 'p' pause/resume, 'q' or Ctrl-C stop")

    if args.dry_run:
        node.get_logger().info("--dry-run: not publishing")
        _restore_switch()
        rclpy.try_shutdown()
        signal.signal(signal.SIGINT, prev_sigint)
        signal.signal(signal.SIGTERM, prev_sigterm)
        return 0

    # ---- helpers --------------------------------------------------------
    def publish_point(q_target, tfs_sec):
        traj = JointTrajectory()
        traj.joint_names = joints
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0
        pt = JointTrajectoryPoint()
        pt.positions = list(q_target)
        sec = int(tfs_sec)
        nsec = int(round((tfs_sec - sec) * 1e9))
        if nsec >= 1_000_000_000:
            sec += 1
            nsec -= 1_000_000_000
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(pt)
        pub.publish(traj)

    # Linear interpolation in the recorded samples. ts is sorted, so we
    # use bisect to find the segment in O(log n) per tick; total work
    # across the whole replay is O(N log N) which is fine even for very
    # long recordings.
    def setpoint_at(t_query):
        if t_query <= t_first:
            return list(qs[0])
        if t_query >= t_last:
            return list(qs[-1])
        i = bisect.bisect_right(ts, t_query)
        # ts[i-1] <= t_query < ts[i]
        t_prev = ts[i - 1]
        t_next = ts[i]
        dt = t_next - t_prev
        if dt <= 0.0:
            return list(qs[i])
        alpha = (t_query - t_prev) / dt
        q_prev = qs[i - 1]
        q_next = qs[i]
        return [q_prev[k] + alpha * (q_next[k] - q_prev[k])
                for k in range(n_dof)]

    # ---- ramp-in: single 1-point traj at samples[0] over `ramp_in` ----
    # During this window the controller's pre-roll segment linearly
    # interpolates from its current hold_pos_ (= measured joint position
    # at activation) to the recorded start pose, so we sidestep a slam if
    # the arm was nowhere near the recorded start.
    restore_kb, key_pressed = _setup_keyboard_listener()
    stop_reason = 'complete'
    try:
        if ramp_in > 0.0:
            node.get_logger().info(f"ramp-in ({ramp_in:.1f}s) to start pose ...")
            publish_point(qs[0], ramp_in)
            ramp_end = time.time() + ramp_in
            while time.time() < ramp_end:
                if stop_flag['val']:
                    stop_reason = 'SIGINT'
                    break
                ch = key_pressed()
                if ch in ('q', 'Q'):
                    stop_reason = "'q' pressed"
                    break
                rclpy.spin_once(node, timeout_sec=0.05)
            if stop_reason != 'complete':
                raise StopIteration  # break out into finally below

        # ---- streaming loop --------------------------------------------
        t_cursor = t_first
        mode = 'playing'
        last_tick_wall = time.time()
        next_publish_wall = time.time()
        last_log_wall = time.time()

        while True:
            if stop_flag['val']:
                stop_reason = 'SIGINT'
                break

            ch = key_pressed()
            if ch in ('q', 'Q'):
                stop_reason = "'q' pressed"
                break
            if ch in ('p', 'P'):
                if mode == 'playing':
                    mode = 'paused'
                    node.get_logger().info(
                        f"paused at t={t_cursor - t_first:.2f}s "
                        f"(of {t_last - t_first:.2f}s). "
                        f"'p' to resume, 'q' to stop")
                else:
                    mode = 'playing'
                    node.get_logger().info(
                        f"resumed at t={t_cursor - t_first:.2f}s")
                # Reset the wall-clock baseline so the cursor doesn't
                # jump forward (or backward) at the mode transition.
                last_tick_wall = time.time()

            now = time.time()
            if now < next_publish_wall:
                rclpy.spin_once(
                    node, timeout_sec=min(0.01, next_publish_wall - now))
                continue
            # Roll the publish deadline forward (drop any backlog so we
            # always aim at "the next future tick", not catch-up bursts).
            while next_publish_wall <= now:
                next_publish_wall += pub_period

            if mode == 'playing':
                dt = now - last_tick_wall
                t_cursor = min(t_cursor + dt * scale, t_last)
            last_tick_wall = now

            publish_point(setpoint_at(t_cursor), tfs)

            # 1 Hz progress log.
            if now - last_log_wall >= 1.0:
                last_log_wall = now
                tag = 'PAUSED' if mode == 'paused' else 'play'
                node.get_logger().info(
                    f"  [{tag}] t={t_cursor - t_first:6.2f}/"
                    f"{t_last - t_first:.2f}s")

            # End of trajectory: only counts when actually playing, so
            # 'p' at the very end doesn't auto-exit.
            if mode == 'playing' and t_cursor >= t_last:
                stop_reason = 'end of trajectory'
                break
    except StopIteration:
        pass
    finally:
        restore_kb()
        # Last publish wins; whatever we sent most recently is the
        # controller's hold target. For 'q'/Ctrl-C/SIGINT, we deliberately
        # do NOT re-publish anything here -- the most recent streamed
        # point is the current setpoint, and re-snapping to it would only
        # add jitter. The motor PD holds there briefly until we restore
        # gravity-comp on the next line, which is the bumpless hand-off
        # (joint_position_controller's on_deactivate zeros kp/kd).
        _restore_switch()
        rclpy.try_shutdown()
        signal.signal(signal.SIGINT, prev_sigint)
        signal.signal(signal.SIGTERM, prev_sigterm)

    print(f"stopped: {stop_reason}")
    return 0


# =============================================================================
# main
# =============================================================================
def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    p_rec = sub.add_parser('record', help='record /joint_states under gravity comp')
    p_rec.add_argument('--output', '-o', required=True,
                       help='YAML output path')
    p_rec.add_argument('--rate', type=float, default=50.0,
                       help='sample rate (Hz, default 50)')
    p_rec.add_argument('--joints', nargs='*', default=[],
                       help='joint names to record (default: all in /joint_states)')
    p_rec.add_argument('--max-duration', type=float, default=120.0,
                       help='max recording length, seconds (default 120)')
    p_rec.set_defaults(func=cmd_record)

    p_rep = sub.add_parser('replay', help='replay through joint_position_controller')
    p_rep.add_argument('input', help='YAML input path (from `record`)')
    p_rep.add_argument('--time-scale', type=float, default=1.0,
                       help='playback speed, 1.0=real-time, 0.5=half (default 1.0)')
    p_rep.add_argument('--ramp-in', type=float, default=2.0,
                       help='seconds to ramp from current pose to recorded '
                            'start (default 2.0); kp is high so do not lower '
                            'this unless the recorded start pose is close to '
                            'the current pose')
    p_rep.add_argument('--publish-rate', type=float, default=50.0,
                       help='setpoint streaming rate, Hz (default 50). Each '
                            'tick the script interpolates the recorded '
                            'samples at the current cursor and publishes a '
                            '1-point trajectory; pausing simply stops '
                            'advancing the cursor so the controller holds.')
    p_rep.add_argument('--command-topic', default='/joint_position_command',
                       help='trajectory topic (default /joint_position_command)')
    p_rep.add_argument('--dry-run', action='store_true',
                       help='build and log the trajectory but do not publish')
    p_rep.add_argument('--auto-switch', dest='auto_switch',
                       action='store_true', default=True,
                       help='before replay, activate '
                            'joint_position_controller, keep '
                            'gravity_compensation_controller active, and '
                            'deactivate cartesian_position_controller if '
                            'active; restore teach mode on exit '
                            '(default ON)')
    p_rep.add_argument('--no-auto-switch', dest='auto_switch',
                       action='store_false',
                       help='do not touch the controller manager '
                            '(legacy: switch controllers by hand)')
    p_rep.add_argument('--position-controller',
                       default='joint_position_controller',
                       help='controller to activate during replay '
                            '(default joint_position_controller)')
    p_rep.add_argument('--gravity-controller',
                       default='gravity_compensation_controller',
                       help='controller to keep active during replay '
                            '(default gravity_compensation_controller)')
    p_rep.add_argument('--cartesian-controller',
                       default='cartesian_position_controller',
                       help='controller to deactivate if active '
                            '(default cartesian_position_controller)')
    p_rep.set_defaults(func=cmd_replay)

    # Strip ROS 2 CLI args (e.g. `--ros-args -r __node:=teach_replayer`
    # injected by launch_ros.actions.Node) before argparse sees them;
    # otherwise argparse aborts with "unrecognized arguments". Same
    # pattern is used in zero_at_current_pose.py / joint_state_translator.py.
    argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = p.parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
