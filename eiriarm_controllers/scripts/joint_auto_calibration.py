#!/usr/bin/env python3
"""Manual reference calibration for all seven joints, one supported joint at a time."""
import argparse
from collections import deque
from dataclasses import asdict
import datetime
import math
import os
from pathlib import Path
import queue
import signal
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from w3_robot_bridge.msg import MotorCommand, MotorCommandArray, MotorStateArray
import yaml

from calibration_direction import DirectionPreview, load_directions
from calibration_seek import ORDER, FrictionModel, PDCommand, SeekLimits, SeekMachine
from joint_zero_calibration import load_calibration_config


COMMANDS = '/w3_robot_bridge_node/commands'
STATE = '/w3_robot_bridge_node/state'
MANUAL_GROUP = ORDER  # Human joints 7, 6, 5, 3, 4, 2, 1; no driven motion.


SKIPPED = 10
MANUAL_EXITED = 11


INSTRUCTIONS = {
    'arm': 'Support every joint; disable other motors/controllers. ENTER enables ONLY this joint; S skips.',
    'enabling': 'Waiting for fresh enabled feedback. ESC aborts.',
    'ready': 'Start near model zero. Clear the path. ENTER starts slow position-PD seeking.',
    'seek': 'Seeking slowly. ESC stops. Do not touch the moving joint.',
    'contact': 'Loaded stop captured; passive rebound is OK. Confirm HARD STOP/path, then ENTER to withdraw.',
    'initial_stall': 'EARLY STALL: output OFF. Reposition then R to retry; ENTER only for a confirmed hard stop; S skips.',
    'return_probe': 'Withdrawing toward zero; enough clearance + settling starts re-probe. Exact zero NOT required.',
    'reseek': 'Second position-PD approach from the withdrawn position. ESC stops.',
    'return_confirm': 'Loaded stops verified; passive rebound is OK. Check path; ENTER withdraws from current pose.',
    'return': 'Withdrawing for passive verification; exact zero NOT required. ESC stops.',
    'manual': 'Model shows EXPECTED limit. Push gently to that stop; hold still, ENTER captures and saves.',
    'manual_preview': 'SAVED: model follows real motion. No return/re-probe. N next; ESC exits and keeps saved result.',
    'verify': 'PASSIVE: hand-move this joint >=5 deg; compare model. ENTER accepts; ESC discards.',
}


def load_controller_gains(path, name, profile):
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError('Invalid controller gains YAML')
    if 'profiles' in data:
        joints = data['joints']
        gains = data['profiles'][profile]
    else:
        gains = data['joint_position_controller']['ros__parameters']
        joints = gains['joints']
    kp, kd = gains['kp_gains'], gains['kd_gains']
    if (not all(isinstance(x, list) for x in (joints, kp, kd))
            or len(set(joints)) != len(joints) or name not in joints
            or len(kp) != len(joints) or len(kd) != len(joints)):
        raise ValueError('Controller gains must uniquely match joint names and array lengths')
    index = joints.index(name)
    values = kp[index], kd[index]
    if any(isinstance(x, bool) or not isinstance(x, (int, float))
           or not math.isfinite(x) or x <= 0 for x in values):
        raise ValueError('Selected controller gains must be finite positive numbers')
    return tuple(float(x) for x in values)


def motion_limits(args, joint, manual):
    if not math.isfinite(args.motion_duration) or not 1 <= args.motion_duration <= 60:
        raise ValueError('--motion-duration must be 1..60 seconds')
    speed = args.speed if args.speed is not None else abs(joint.urdf_pos_at_limit) / args.motion_duration
    stall_torque = (args.stall_torque if args.stall_torque is not None else
                    1.0 if joint.slot == 5 and not manual else 0.5)
    higher_abort = not manual and (joint.slot == 2 or (joint.slot == 5 and stall_torque > 0.5))
    common = dict(stall_torque=stall_torque, speed=0.025 if manual else speed,
                  target_tolerance=args.target_tolerance,
                  return_min_fraction=args.return_min_fraction,
                  effort_abort=(args.effort_abort if args.effort_abort is not None else
                                1.5 if higher_abort else 1.0),
                  max_travel=min(2 * math.pi, abs(joint.urdf_pos_at_limit) + 0.35))
    if manual:
        return SeekLimits(**common), 'manual: gains are not applied'
    if (args.cal_kp is None) != (args.cal_kd is None):
        raise ValueError('Specify --cal-kp and --cal-kd together, or omit both to load controller gains')
    if args.cal_kp is None:
        if args.gains_yaml is None:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory('eiriarm_controllers')) / 'config/teleop_joint_gains.yaml'
        else:
            path = args.gains_yaml.expanduser().resolve()
        kp, kd = load_controller_gains(path, joint.name, args.gain_profile)
        source = f'{path} (profile={args.gain_profile}, joint={joint.name})'
    else:
        kp, kd = args.cal_kp, args.cal_kd
        source = 'explicit --cal-kp/--cal-kd overrides'
    if not math.isfinite(kp) or kp <= 0:
        raise ValueError('kp must be finite and positive')
    # Preserve the spring-effort budget when using stiffer production gains.
    lead = args.max_position_error
    if lead is None:
        lead = 1.2 * stall_torque / kp
    return SeekLimits(kp=kp, kd=kd, max_position_error=lead,
                      max_speed=args.max_speed if args.max_speed is not None else max(0.30, 1.6 * speed),
                      hard_max_speed=args.hard_max_speed if args.hard_max_speed is not None else max(0.50, 2 * speed),
                      **common), source


def load_calibration_friction(args, joint, manual):
    if manual or not args.friction_compensation:
        return FrictionModel(), 'disabled (manual or explicit option)'
    from ament_index_python.packages import get_package_share_directory
    config = args.controller_config or Path(get_package_share_directory('eiriarm_controllers')) / 'config/dual_arm_controllers.yaml'
    path = args.friction_yaml or args.calibration_yaml.expanduser().resolve().with_name('friction_model.yaml')
    params = yaml.safe_load(config.expanduser().read_text())['gravity_compensation_controller']['ros__parameters']
    names, motors = params['joints'], params['motor_types']
    gains = params.get('friction_gains') or [params['friction_gain']] * len(names)
    if len(set(names)) != len(names) or len(motors) != len(names) or len(gains) != len(names) or joint.name not in names:
        raise ValueError('Friction controller joint/motor/gain arrays are inconsistent')
    index = names.index(joint.name)
    if motors[index] != joint.motor_type:
        raise ValueError(f'Motor type mismatch for {joint.name}: {motors[index]} vs {joint.motor_type}')
    entry = yaml.safe_load(path.expanduser().read_text())[joint.motor_type]
    model = FrictionModel(entry['coulomb_pos'], entry['coulomb_neg'], entry['viscous'], gains[index])
    model.validate()
    return model, f'{path.resolve()} + {config.resolve()} ({joint.name}, {joint.motor_type}; raw motor direction)'


def prepare(args):
    if args.auto_group:
        raise ValueError('Automatic calibration is disabled; remove --auto-group to use manual calibration')
    source = args.calibration_yaml.expanduser().resolve()
    direction_path = args.directions_yaml.expanduser().resolve()
    cfg = load_calibration_config(source)
    data = yaml.safe_load(source.read_text())
    load_directions(yaml.safe_load(direction_path.read_text()), cfg)
    if len(cfg.joints) != 7 or {j.slot for j in cfg.joints} != set(range(7)):
        raise ValueError('Require a complete seven-joint arm configuration')
    output = (args.output or source.with_name(
        'joint_offsets_' + ('left' if cfg.channel == 0 else 'right') + '.yaml')).expanduser().resolve()
    if output in (source, direction_path):
        raise ValueError('Output must be separate from references and directions')
    previous_text = output.read_text() if output.exists() else None
    previous = yaml.safe_load(previous_text) if previous_text else {'channel': cfg.channel, 'offsets': []}
    if previous.get('channel') != cfg.channel:
        raise ValueError('Existing offset file belongs to another arm')
    entries = previous.get('offsets', [])
    by_slot = {j.slot: j for j in cfg.joints}
    done = set()
    for entry in entries:
        slot = entry['slot']
        if slot in done or slot not in by_slot:
            raise ValueError('Duplicate/invalid joint in existing offset file')
        if entry['name'] != by_slot[slot].name or entry.get('axis_sign') != by_slot[slot].axis_sign:
            raise ValueError('Existing offsets conflict with verified names/directions')
        if not math.isfinite(float(entry['zero_offset'])):
            raise ValueError('Existing offset is invalid')
        done.add(slot)
    slot = args.joint - 1 if args.joint else MANUAL_GROUP[0]
    joint = by_slot[slot]
    entry = next(e for e in data['joints'] if e['slot'] == slot)
    reviewed = entry.get('reference_review', {})
    if (reviewed.get('confirmed') is not True or not math.isfinite(joint.urdf_pos_at_limit)
            or reviewed.get('confirmed_angle_rad') != joint.urdf_pos_at_limit):
        raise ValueError('Run --mode limit-preview and use its reviewed YAML first')
    engine = SeekMachine(joint.axis_sign, joint.urdf_pos_at_limit, joint.limit_side, SeekLimits(), manual=True)
    engine.gain_source = 'manual: gains are not applied'
    engine.friction_source = 'disabled: manual calibration has no torque feed-forward'
    return cfg, joint, engine, output, previous, previous_text


class SingleJointNode(Node):
    def __init__(self, channel, joint, limits, friction=None, manual_only=False):
        super().__init__('joint_auto_calibration')
        self.channel, self.joint = channel, joint
        self.limits, self.timeout = limits, limits.feedback_timeout
        self.friction = friction if friction is not None else FrictionModel()
        self.manual_only = manual_only
        self.sample = None
        self.received = 0.0
        self.enabled = False
        self.error = None
        self.other_enabled = False
        self.publisher = self.create_publisher(MotorCommandArray, COMMANDS, 10)
        self.subscription = self.create_subscription(MotorStateArray, STATE, self.on_state,
                                                     qos_profile_sensor_data)

    def on_state(self, msg):
        self.other_enabled = any(m.online and m.enabled and (m.channel, m.motor_index) !=
                                 (self.channel, self.joint.slot) for m in msg.motors)
        found = next((m for m in msg.motors if (m.channel, m.motor_index) ==
                      (self.channel, self.joint.slot)), None)
        self.sample = None
        if found and found.online and all(math.isfinite(x) for x in
                                         (found.position, found.velocity, found.torque)):
            self.sample = (found.position, found.velocity, found.torque)
            self.received = time.monotonic()
            self.error, self.enabled = found.error_flags, found.enabled

    def fresh(self):
        return self.sample if time.monotonic() - self.received <= self.timeout else None

    def exclusive(self):
        return (self.count_publishers(COMMANDS) == 1
                and self.count_publishers('/w3_robot_bridge_node/command') == 0
                and self.count_publishers(STATE) == 1 and not self.other_enabled)

    def send(self, command=None, mode=MotorCommand.MODE_RUN):
        command = command if command is not None else PDCommand()
        values = asdict(command)
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError('Calibration PD commands must be finite')
        if getattr(self, 'manual_only', False) and (command.active or command.velocity != 0 or command.torque != 0):
            raise ValueError('Manual-only calibration refuses nonzero gains, velocity or torque')
        if command.active:
            sample = self.fresh()
            if (mode != MotorCommand.MODE_RUN or sample is None
                    or command.kp != self.limits.kp or command.kd != self.limits.kd
                    or abs(command.position - sample[0]) > self.limits.max_position_error + 1e-6
                    or abs(command.velocity) > min(self.limits.speed, 0.2 / self.limits.kd) + 1e-9
                    or abs(command.torque) > self.friction.maximum(self.limits.speed) + 1e-9
                    or abs(command.position) >= self.joint.pos_max - 0.05):
                raise ValueError('Refusing invalid, stale or out-of-bounds PD command')
        elif command.kp != 0 or command.kd != 0 or command.velocity != 0 or command.torque != 0:
            raise ValueError('Passive calibration command must have zero gains and velocity')
        msg = MotorCommandArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.commands = [MotorCommand(channel=self.channel, motor_index=self.joint.slot,
                                    mode=mode, **{k: float(v) for k, v in values.items()})]
        self.publisher.publish(msg)

    def disable_confirmed(self):
        start = time.monotonic()
        last_disable = 0.0
        while rclpy.ok() and time.monotonic() - start < 3.0:
            self.send()
            if time.monotonic() - last_disable >= 0.2:
                self.send(mode=MotorCommand.MODE_DISABLE)
                last_disable = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.received > start and self.fresh() is not None and self.error == 0 and not self.enabled:
                return True
            time.sleep(0.01)
        return False


def save_result(output, previous, previous_text, joint, engine, channel):
    valid_phase = engine.state == 'done' or (engine.manual and engine.state == 'manual_preview')
    if (not valid_phase or engine.offset is None or engine.stop_raw is None
            or not all(math.isfinite(x) for x in (engine.offset, engine.stop_raw))
            or not math.isclose(joint.axis_sign * (engine.stop_raw - engine.offset),
                                joint.urdf_pos_at_limit, abs_tol=1e-9)):
        raise ValueError('Cannot save an unverified or inconsistent reference')
    current = output.read_text() if output.exists() else None
    if current != previous_text:
        raise RuntimeError('Offset file changed during calibration; refusing to overwrite it')
    entry = dict(name=joint.name, slot=joint.slot, motor_type=joint.motor_type,
                 axis_sign=joint.axis_sign, direction_verified=True,
                 zero_offset=engine.offset, raw_at_reference=engine.stop_raw,
                 urdf_pos_at_reference=joint.urdf_pos_at_limit, limit_side=joint.limit_side,
                 reference_confirmed=True, visually_verified=not engine.manual,
                 method='manual' if engine.first_stop is None else 'repeated_position_pd_seek',
                 motion_settings=asdict(engine.limits),
                 return_duration=engine.return_duration,
                 retreats=engine.retreats,
                 gain_source=engine.gain_source,
                 friction_source=engine.friction_source, friction_model=asdict(engine.friction),
                 identified_at=datetime.datetime.now().isoformat(timespec='seconds'))
    entries = [e for e in previous['offsets'] if e['slot'] != joint.slot] + [entry]
    result = dict(channel=channel, mode='assisted-hard-stop',
                  complete=len(entries) == 7, offsets=sorted(entries, key=lambda e: e['slot']))
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=output.parent, delete=False) as stream:
            temporary = Path(stream.name)
            yaml.safe_dump(result, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output)
    finally:
        if temporary and temporary.exists():
            temporary.unlink()


def preview_angle(joint, engine, phase, sample, origin):
    if phase != 'active' or sample is None or (engine.manual and engine.offset is None):
        return joint.urdf_pos_at_limit
    return joint.axis_sign * (sample[0] - (engine.offset if engine.offset is not None else origin))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration-yaml', type=Path, required=True)
    parser.add_argument('--directions-yaml', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--joint', type=int, choices=range(1, 8), help='Only this joint (1..7); default visits all seven manually')
    group_parser = parser.add_mutually_exclusive_group()
    group_parser.add_argument('--auto-group', action='store_true',
                        help='Removed: automatic calibration is disabled')
    group_parser.add_argument('--manual-group', action='store_true',
                        help='Compatibility alias for the default manual 7-6-5-3-4-2-1 group')
    parser.add_argument('--manual', action='store_true', help='Compatibility alias: all calibration is manual')
    parser.add_argument('--rate', type=float, default=100.0, help='Zero-impedance feedback command rate, 100..350 Hz')
    parser.add_argument('--diagnostic-dir', type=Path,
                        default=Path('/tmp/eiriarm_calibration_diagnostics'),
                        help='Save recent samples on a fault; does not contain usable offsets')
    args = parser.parse_args(argv)
    if not math.isfinite(args.rate) or not 100 <= args.rate <= 350:
        parser.error('--rate must be between 100 and 350 Hz')
    if args.auto_group:
        parser.error('Automatic calibration is disabled; remove --auto-group to use manual calibration')
    selection = [slot + 1 for slot in MANUAL_GROUP] if args.joint is None else [args.joint]
    for index, selected in enumerate(selection):
        selected_args = argparse.Namespace(**{**vars(args), 'joint': selected})
        result = run_single(selected_args)
        if result == MANUAL_EXITED:
            return 0
        if result not in (0, SKIPPED):
            return result
        if index + 1 < len(selection):
            print('Preparing next joint; ENTER enables it, S skips it.', flush=True)
    print('Manual calibration finished (saved or explicitly skipped).')
    return 0


def run_single(args):
    try:
        cfg, joint, engine, output, previous, previous_text = prepare(args)
    except (OSError, KeyError, ValueError, TypeError) as exc:
        print(f'Preflight rejected: {exc}')
        return 2
    print(f'can{cfg.channel} ONLY joint {joint.slot + 1} ({joint.name}); MANUAL; no driven motion')
    print(f'Gain source: {engine.gain_source}')
    print(f'Friction source: {engine.friction_source}; parameters={asdict(engine.friction)}')
    if any(entry['slot'] == joint.slot for entry in previous['offsets']):
        print('Existing result found: ENTER recalibrates; S skips and keeps the existing result.')
    print(f'Feedback/command rate: {args.rate:g} Hz (non-realtime scheduling)')
    print('MANUAL: zero kp/kd, velocity and torque feed-forward throughout; no automatic motion.')
    print('ENTER enables feedback; model stays at expected limit until ENTER captures and saves it.')
    print('After saving, hand-move to inspect. N advances; ESC exits without undoing the saved result.')
    print('Stop all controllers and other command publishers. Support ALL joints; disable other motors.')
    if joint.slot == 3:
        print('JOINT 4: manually reposition and support joint 3 to clear the bracket BEFORE enabling.')
    stop = threading.Event()
    rclpy.init()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    node = SingleJointNode(cfg.channel, joint, engine.limits, engine.friction, manual_only=True)
    preview = None
    attempted = accepted = disabled = skipped = False
    manual_saved = False
    exit_requested = False
    phase = 'arm'
    last_enable = last_log = last_preview = 0.0
    armed_at = 0.0
    origin = None
    recent = deque(maxlen=300)
    try:
        preview = DirectionPreview(node, cfg, motion_workflow=True)
        preview.wait_ready(stop)
        period = 1.0 / args.rate
        next_tick = time.monotonic()
        while rclpy.ok() and not stop.is_set():
            # Drain feedback between ticks; processing only one callback per tick
            # would queue up the faster bridge state stream.
            rclpy.spin_once(node, timeout_sec=0.0)
            while rclpy.ok() and not stop.is_set():
                remaining = next_tick - time.monotonic()
                if remaining <= 0:
                    break
                rclpy.spin_once(node, timeout_sec=min(remaining, 0.002))
            if not rclpy.ok() or stop.is_set():
                break
            if preview.process.poll() is not None:
                if manual_saved and preview.process.returncode == 0:
                    exit_requested = True
                    break
                raise RuntimeError('Preview closed; calibration aborted')
            now = time.monotonic()
            try:
                key = preview.keys.get_nowait()
            except queue.Empty:
                key = None
            if key == 'escape':
                exit_requested = True
                break
            if key == 's':
                if manual_saved:
                    accepted = True
                    break
                skipped = True
                break
            if phase == 'arm':
                if key == 'enter':
                    if not node.exclusive():
                        raise RuntimeError('Require one bridge, no other command publishers or enabled motors')
                    phase, armed_at, attempted = 'enabling', now, True
            if phase == 'enabling':
                if not node.exclusive():
                    raise RuntimeError('Command ownership or motor isolation lost')
                node.send()
                if now - last_enable >= 0.2:
                    node.send(mode=MotorCommand.MODE_ENABLE)
                    last_enable = now
                if node.received > armed_at and node.fresh() is not None and node.error == 1 and node.enabled:
                    phase = 'active'
                    origin = node.fresh()[0]
                    while not preview.keys.empty():
                        preview.keys.get_nowait()
                    key = None
                elif now - armed_at > 5:
                    raise RuntimeError('Enable confirmation timed out')
            if phase == 'active':
                if not node.exclusive():
                    engine.fault('command_ownership_or_motor_isolation_lost')
                elif node.error != 1 or not node.enabled:
                    engine.fault('motor_disabled_or_error')
                sample = node.fresh()
                if sample and abs(sample[0]) >= joint.pos_max - 0.1:
                    engine.fault('encoder_near_wrap_boundary')
                command = engine.update(now, sample, key)
                node.send(command)
                recent.append(dict(time=now, phase=engine.state, sample=sample,
                                   filtered_velocity=engine.filtered_velocity,
                                   residual_effort=engine.residual_effort,
                                   command=asdict(command)))
                if engine.state == 'fault':
                    raise RuntimeError(engine.reason)
                if engine.state == 'manual_preview' and not manual_saved:
                    save_result(output, previous, previous_text, joint, engine, cfg.channel)
                    manual_saved = True
                    print(f'Manual reference SAVED for joint {joint.slot + 1} to {output}. '
                          'Live preview only; N next, ESC exits and keeps this result.', flush=True)
                if engine.state == 'done':
                    accepted = True
                    break
            state = phase if phase != 'active' else engine.state
            if now - last_preview > 0.04:
                sample = node.fresh()
                q = preview_angle(joint, engine, phase, sample, origin)
                preview.pose_pub.publish(JointState(name=[joint.name], position=[q]))
                extra = ('Joint 4: reposition/support joint 3; avoid the bracket.\n' if joint.slot == 3 else '')
                footer = ('Reference already saved. N/S next; ESC exits and keeps it.' if manual_saved else
                          'S skips this joint without saving. ESC aborts the group; support the real arm.')
                control_text = ('MANUAL: kp=kd=v_ff=tau_ff=0; no driven motion.' if engine.manual else
                                f'PD kp={engine.command.kp:.1f} kd={engine.command.kd:.1f}; '
                                f'residual stall threshold={engine.limits.stall_torque:.2f} Nm (NOT output cap)')
                preview.text_pub.publish(String(data=(
                    f'JOINT CALIBRATION  |  can{cfg.channel}  |  Joint {joint.slot + 1}  |  {state.upper()}\n'
                    f'Reference: {math.degrees(joint.urdf_pos_at_limit):+.2f} deg; '
                    f'{joint.limit_side}; sign={joint.axis_sign:+d}\n'
                    f'{control_text}\n'
                    f'{INSTRUCTIONS.get(state, state)}\n{extra}'
                    f'{engine.reason}\n'
                    f'{footer}')))
                last_preview = now
            if now - last_log > 1.0:
                sample = node.fresh()
                arrival = (f'goal={engine.goal:+.5f} error={engine.goal - sample[0]:+.5f} '
                           f'clearance={engine.retreat_progress(sample[0])[0]:.5f}/'
                           f'{engine.retreat_progress(sample[0])[1]:.5f} '
                           if engine.goal is not None and sample is not None else '')
                print(f'{state}: raw={node.fresh()} q_cmd={engine.command.position:+.5f} '
                      f'v_cmd={engine.command.velocity:+.4f} kp={engine.command.kp} kd={engine.command.kd} '
                      f'v_filtered={engine.filtered_velocity:+.4f} '
                      f'tau_ff={engine.command.torque:+.4f} residual={engine.residual_effort:+.4f} '
                      f'{arrival}{engine.reason}', flush=True)
                last_log = now
            next_tick = max(next_tick + period, time.monotonic())
    except Exception as exc:
        print(f'Calibration stopped: {exc}', flush=True)
        if engine.fault_snapshot is not None:
            print(f'Fault snapshot: {engine.fault_snapshot}', flush=True)
    finally:
        if attempted:
            try:
                disabled = node.disable_confirmed()
            except Exception as exc:
                print(f'Disable failed: {exc}')
            if not disabled:
                print('WARNING: DISABLE NOT CONFIRMED. Support the joint and use the hardware stop.')
        if preview is not None:
            preview.close()
        node.destroy_node()
        rclpy.shutdown()
    if skipped:
        if attempted and not disabled:
            print('Skip failed: disable not confirmed. Group stopped; no result saved.')
            return 1
        print(f'Skipped joint {joint.slot + 1}; existing calibration file unchanged.')
        return SKIPPED
    if engine.fault_snapshot is not None:
        try:
            args.diagnostic_dir.mkdir(parents=True, exist_ok=True)
            diagnostic = args.diagnostic_dir / (
                datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f') + f'_can{cfg.channel}_joint{joint.slot + 1}.yaml')
            diagnostic.write_text(yaml.safe_dump(dict(reason=engine.reason,
                invocation=sys.argv, script_path=str(Path(__file__).resolve()),
                gain_source=engine.gain_source, motion_settings=asdict(engine.limits),
                return_duration=engine.return_duration,
                retreats=engine.retreats,
                friction_model=asdict(engine.friction), friction_source=engine.friction_source,
                fault=engine.fault_snapshot, recent=list(recent)), sort_keys=False))
            print(f'Diagnostic samples saved to {diagnostic}')
        except OSError as exc:
            print(f'Could not save diagnostic samples: {exc}')
    if manual_saved:
        print(f'Manual reference remains saved to {output}; motor disable confirmed={disabled}.')
        if accepted and disabled:
            return 0
        if disabled and engine.state != 'fault' and (exit_requested or stop.is_set()):
            return MANUAL_EXITED
        return 1
    if accepted and disabled:
        try:
            save_result(output, previous, previous_text, joint, engine, cfg.channel)
        except (OSError, ValueError, RuntimeError) as exc:
            print(f'Result not saved: {exc}')
            return 2
        print(f'Saved joint {joint.slot + 1} to {output}. Motor disable confirmed.')
        return 0
    print('No calibration result saved.')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
