#!/usr/bin/env python3
"""Manual joint zero calibration.

Unified equation:
        zero_offset = raw_at_reference - axis_sign * urdf_pos_at_reference
        => urdf_pos = axis_sign * (raw_pos - zero_offset)
        => raw_cmd  = axis_sign * urdf_cmd + zero_offset

Modes (CLI flag --mode):

    direction: FK-only MuJoCo preview, model zero plus one motor's delta.
        SPACE flips the displayed direction, ENTER captures/accepts.
        Saves a separate direction file; does not calibrate absolute zero.

    hard-stop  (default, suited for arms without a zero-pose fixture):
        Sequentially per joint:
          - prompt 'push joint X into its single-side hard stop, ENTER'
          - capture median raw at that limit
          - zero_offset = raw_at_limit - axis_sign * urdf_pos_at_limit
        urdf_pos_at_limit comes from CAD/drawings (the URDF position the
        link physically reaches at that hard stop, NOT necessarily the
        URDF <limit> field).

    zero-pose  (for arms with a flat-surface / fixture / cube that puts
        every link in URDF zero):
          - put the whole arm in URDF zero pose, press ENTER ONCE
          - all joints captured simultaneously
          - zero_offset = raw_at_zero  (since urdf_pos_at_reference = 0)
        Per-joint 'limit_side' and 'urdf_pos_at_limit' are ignored in this
        mode (you can omit them).

Usage:

    # bring the bus up first (terminal A):
    ros2 launch eiriarm_bringup bridge.launch.py

    # zero-pose calibration (with fixture):
    ros2 run eiriarm_controllers joint_zero_calibration \\
        --mode zero-pose \\
        --calibration-yaml joint_calibration.yaml \\
        --output joint_offsets_dual.yaml

    # hard-stop calibration (dual 7-DoF arms, no fixture):
    ros2 run eiriarm_controllers joint_zero_calibration \\
        --mode hard-stop \\
        --calibration-yaml joint_calibration.yaml \\
        --output joint_offsets_dual.yaml

Safety:
    * Motors are enabled in zero-impedance MIT mode (kp=0, kd=0, tau_ff=0).
      They will freewheel; gravity-loaded links may drop. Be ready to
      support the arm before pressing ENTER.
    * Ctrl-C attempts to disable all selected motors and checks feedback.
      Missing feedback is NOT proof of disable; support the arm throughout.
    * If the arm is moving when you press ENTER (|max - min| over the
      window > sampling.max_motion_rad), the sample is rejected and the
      script asks you to try again -- avoids capturing a bouncy reading.
"""
import argparse
import dataclasses
import datetime
import math
import signal
import statistics
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from w3_robot_bridge.msg import (
    MotorCommand,
    MotorCommandArray,
    MotorStateArray,
)
from calibration_direction import DirectionPreview, load_directions


MOTOR_LIMITS: Dict[str, Tuple[float, float, float]] = {
    # type:    (pos_max, vel_max, tor_max) -- must match dm_motors_*.yaml
    'DM4310':  (12.5, 50.0, 10.0),
    'DM4340':  (12.5, 20.0, 28.0),
    'DM4340P': (12.5, 20.0, 28.0),
    'DM8009':  (12.5, 45.0, 54.0),
}

NO_DATA_EPSILON = 1e-3
LIMIT_SIDES = ('positive', 'negative')

# DM motor state code (MotorState.error_flags = RX byte0 high 4 bits).
# 0=disabled, 1=enabled(MIT), 8=overvoltage, 9=undervoltage,
# 10=overcurrent, 11=MOS overtemp, 12=rotor overtemp,
# 13=comm lost, 14=overload.
DM_ERR_ENABLED = 1
DM_ERR_NAMES: Dict[int, str] = {
    0: 'disabled',
    1: 'enabled',
    8: 'overvoltage',
    9: 'undervoltage',
    10: 'overcurrent',
    11: 'MOS-overtemp',
    12: 'rotor-overtemp',
    13: 'comm-lost',
    14: 'overload',
}


def _err_name(err: Optional[int]) -> str:
    if err is None:
        return 'no-data'
    return DM_ERR_NAMES.get(err, f'unknown({err})')


@dataclasses.dataclass
class JointSpec:
    slot: int
    name: str
    motor_type: str
    # Used only in hard-stop mode. zero-pose mode treats reference as URDF zero
    # so these can be left as defaults.
    limit_side: str  # 'positive' | 'negative' | 'unused'
    urdf_pos_at_limit: float
    pos_max: float
    vel_max: float
    tor_max: float
    axis_sign: int = 1
    direction_verified: bool = False
    reference_original: Optional[float] = None
    reference_confirmed: bool = False


@dataclasses.dataclass
class CalibConfig:
    channel: int
    window_sec: float
    min_samples: int
    max_motion_rad: float
    joints: List[JointSpec]


def load_calibration_config(path: Path) -> CalibConfig:
    if not path.exists():
        raise FileNotFoundError(f'{path} not found')
    with path.open() as f:
        data = yaml.safe_load(f) or {}
    if 'channel' not in data:
        raise KeyError(f'{path}: top-level "channel" missing')
    if 'joints' not in data or not data['joints']:
        raise KeyError(f'{path}: "joints" missing or empty')

    if int(data['channel']) not in (0, 1):
        raise ValueError('W3 arm channel must be 0 (left) or 1 (right)')

    sampling = data.get('sampling', {})
    window_sec = float(sampling.get('window_sec', 0.5))
    min_samples = int(sampling.get('min_samples', 20))
    max_motion = float(sampling.get('max_motion_rad', 0.05))

    seen_slots = set()
    joints: List[JointSpec] = []
    for j in data['joints']:
        for k in ('slot', 'name', 'motor_type', 'limit_side',
                  'urdf_pos_at_limit'):
            if k not in j:
                raise KeyError(f'{path}: joint missing field "{k}"; got {j}')
        slot = int(j['slot'])
        if not 0 <= slot <= 7:
            raise ValueError(f'slot must be 0..7, got {slot}')
        if slot in seen_slots:
            raise ValueError(f'duplicate slot {slot} in {path}')
        seen_slots.add(slot)
        mtype = str(j['motor_type'])
        if mtype not in MOTOR_LIMITS:
            raise ValueError(
                f'unknown motor_type "{mtype}" for slot {slot}; '
                f'expected one of {sorted(MOTOR_LIMITS.keys())}')
        # limit_side / urdf_pos_at_limit are required for hard-stop mode but
        # optional for zero-pose mode -- defer the mode-specific check to main.
        side = str(j.get('limit_side', 'unused')).lower()
        if side not in LIMIT_SIDES + ('unused',):
            raise ValueError(
                f'limit_side must be one of {LIMIT_SIDES} or omitted, '
                f'got "{side}"')
        urdf_pos = float(j.get('urdf_pos_at_limit', 0.0))
        pos_max, vel_max, tor_max = MOTOR_LIMITS[mtype]
        joints.append(JointSpec(
            slot=slot, name=str(j['name']), motor_type=mtype,
            limit_side=side,
            urdf_pos_at_limit=urdf_pos,
            pos_max=pos_max, vel_max=vel_max, tor_max=tor_max))

    return CalibConfig(
        channel=int(data['channel']),
        window_sec=window_sec, min_samples=min_samples,
        max_motion_rad=max_motion, joints=joints)


class CalibrationNode(Node):
    """Keeps motors in zero-impedance mode and exposes latest state per slot."""

    def __init__(self, channel: int, joints: List[JointSpec]):
        super().__init__('joint_zero_calibration')
        if not joints:
            raise ValueError('joints list must not be empty')
        self.channel = channel
        self.joints: List[JointSpec] = joints
        self._slot_to_joint: Dict[int, JointSpec] = {j.slot: j for j in joints}

        self._cmd_pub = self.create_publisher(
            MotorCommandArray, '/w3_robot_bridge_node/commands', 10)
        self._state_sub = self.create_subscription(
            MotorStateArray, '/w3_robot_bridge_node/state',
            self._on_state, qos_profile_sensor_data)

        self._state_lock = threading.Lock()
        # slot -> (pos, vel, tor, stamp_ns)
        self._states: Dict[int, Tuple[float, float, float, int]] = {}
        # ring buffer for sampling: slot -> list[(stamp_ns, pos)]
        # protected by _state_lock; cleared / re-armed by main thread.
        self._sample_buf: Dict[int, List[Tuple[int, float]]] = {
            j.slot: [] for j in joints}
        self._sampling_active = False

        # slot -> latest MotorState.error_flags (only updated for non-sentinel frames).
        # Used by enable_all / disable_all to confirm motor state directly.
        self._state_err: Dict[int, int] = {}
        self._state_received_at: Dict[int, float] = {}

        self._control_timer = self.create_timer(0.010, self._tick_cmd)

        self.get_logger().info(
            f'Calibration node ready on ch{channel}, '
            f'tracking {len(joints)} joint(s):')
        for j in joints:
            self.get_logger().info(
                f'  slot {j.slot} [{j.motor_type}] {j.name}: '
                f'limit_side={j.limit_side}  '
                f'urdf_pos_at_limit={j.urdf_pos_at_limit:+.4f} rad')

    # --------------- ROS callbacks ---------------

    def _on_state(self, msg: MotorStateArray) -> None:
        motors = {m.motor_index: m for m in msg.motors
                  if m.channel == self.channel and m.online
                  and all(math.isfinite(v) for v in (m.position, m.velocity, m.torque))}
        stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                    + msg.header.stamp.nanosec)
        received_at = time.monotonic()
        # Only finite, online W3 states on this arm are admitted above.
        with self._state_lock:
            for j in self.joints:
                if j.slot not in motors:
                    self._states.pop(j.slot, None)
                    self._state_err.pop(j.slot, None)
                    self._state_received_at.pop(j.slot, None)
                    continue
                m = motors[j.slot]
                pos = float(m.position)
                self._states[j.slot] = (
                    pos, float(m.velocity), float(m.torque), stamp_ns)
                self._state_err[j.slot] = int(m.error_flags)
                self._state_received_at[j.slot] = received_at
                if self._sampling_active:
                    self._sample_buf[j.slot].append((stamp_ns, pos))

    def _tick_cmd(self) -> None:
        # Always publish a zero-impedance cmd to every tracked motor.
        # DM motors are passive in MIT mode and only emit feedback when
        # they receive a cmd (this matches friction_compensation_demo.py).
        arr = MotorCommandArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.commands = [
            MotorCommand(channel=self.channel, motor_index=j.slot, mode=MotorCommand.MODE_RUN, position=0.0, velocity=0.0,
                         kp=0.0, kd=0.0, torque=0.0)
            for j in self.joints
        ]
        self._cmd_pub.publish(arr)

    def _publish_enable_msg(self, enable: bool) -> None:
        msg = MotorCommandArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        mode = MotorCommand.MODE_ENABLE if enable else MotorCommand.MODE_DISABLE
        msg.commands = [
            MotorCommand(channel=self.channel, motor_index=slot, mode=mode)
            for slot in [j.slot for j in self.joints]
        ]
        self._cmd_pub.publish(msg)

    # --------------- main-thread helpers ---------------

    def get_state(self, slot: int) -> Optional[Tuple[float, float, float]]:
        with self._state_lock:
            s = self._states.get(slot)
            if time.monotonic() - self._state_received_at.get(slot, 0.0) > 0.5:
                return None
        return None if s is None else (s[0], s[1], s[2])

    def have_any_state(self) -> bool:
        with self._state_lock:
            return len(self._states) > 0

    def begin_sampling(self, slot: int) -> None:
        with self._state_lock:
            self._sample_buf[slot] = []
            self._sampling_active = True

    def begin_sampling_all(self) -> None:
        """Clear every slot's buffer and arm sampling atomically.
        Use this for synchronous multi-joint capture (zero-pose mode)."""
        with self._state_lock:
            for slot in list(self._sample_buf.keys()):
                self._sample_buf[slot] = []
            self._sampling_active = True

    def end_sampling(self, slot: int) -> List[Tuple[int, float]]:
        with self._state_lock:
            self._sampling_active = False
            buf = list(self._sample_buf.get(slot, ()))
            self._sample_buf[slot] = []
        return buf

    # --------------- enable / disable ---------------

    def _snapshot_err(self, since: float = 0.0) -> Dict[int, Optional[int]]:
        """Only admit recent online feedback received after this operation began."""
        cutoff = max(since, time.monotonic() - 0.5)
        with self._state_lock:
            return {j.slot: (self._state_err.get(j.slot)
                            if self._state_received_at.get(j.slot, 0.0) > cutoff
                            else None) for j in self.joints}

    def _format_err_table(self, errs: Dict[int, Optional[int]]) -> str:
        return ', '.join(
            f'slot{s}={_err_name(errs.get(s))}' for s in sorted(errs))

    def enable_all(self, timeout: float = 5.0,
                   stop_evt: Optional[threading.Event] = None) -> bool:
        """Publish ENABLE at 5 Hz until every tracked motor reports
        MotorState.error_flags == DM_ERR_ENABLED, or the timeout expires.
        Returns True iff every joint confirmed enabled."""
        started_at = time.monotonic()
        deadline = started_at + timeout
        last_pub = 0.0
        slot_str = ','.join(str(j.slot) for j in self.joints)
        self.get_logger().info(
            f'ch{self.channel}.id[{slot_str}] ENABLE: 5Hz publish until '
            f'every motor reports err=={DM_ERR_ENABLED} (enabled) '
            f'(timeout {timeout:.1f}s)...')
        while time.monotonic() < deadline:
            if stop_evt is not None and stop_evt.is_set():
                return False
            now = time.monotonic()
            if now - last_pub >= 0.2:
                self._publish_enable_msg(True)
                last_pub = now
            errs = self._snapshot_err(started_at)
            if all(e == DM_ERR_ENABLED for e in errs.values()):
                self.get_logger().info(
                    f'ch{self.channel}.id[{slot_str}] ENABLED '
                    f'(all {len(self.joints)} motors report err=1)')
                return True
            time.sleep(0.02)
        # timeout: log per-slot state for diagnosis
        errs = self._snapshot_err(started_at)
        bad = {s: e for s, e in errs.items() if e != DM_ERR_ENABLED}
        self.get_logger().error(
            f'ENABLE timed out after {timeout:.1f}s; '
            f'slot(s) not enabled: {self._format_err_table(bad)}')
        return False

    def disable_all(self, timeout: float = 3.0,
                    stop_evt: Optional[threading.Event] = None) -> bool:
        """Publish DISABLE at 5 Hz until every tracked motor reports
        MotorState.error_flags != DM_ERR_ENABLED (i.e. disabled or in an error
        state), or the timeout expires. Returns True iff confirmed."""
        # Burst a few zero-cmd frames first so the motors are passive when
        # DISABLE lands. Keep the 10 ms cmd ticker running -- we need the
        # motors to keep responding so we can read fresh err codes.
        arr = MotorCommandArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.commands = [
            MotorCommand(channel=self.channel, motor_index=j.slot, mode=MotorCommand.MODE_RUN, position=0.0, velocity=0.0,
                         kp=0.0, kd=0.0, torque=0.0)
            for j in self.joints
        ]
        for _ in range(3):
            self._cmd_pub.publish(arr)
            time.sleep(0.01)

        started_at = time.monotonic()
        deadline = started_at + timeout
        last_pub = 0.0
        slot_str = ','.join(str(j.slot) for j in self.joints)
        self.get_logger().info(
            f'ch{self.channel}.id[{slot_str}] DISABLE: 5Hz publish until '
            f'every motor reports err!={DM_ERR_ENABLED} '
            f'(timeout {timeout:.1f}s)...')
        while time.monotonic() < deadline:
            if stop_evt is not None and stop_evt.is_set():
                self._publish_enable_msg(False)
                return False
            now = time.monotonic()
            if now - last_pub >= 0.2:
                self._publish_enable_msg(False)
                last_pub = now
            errs = self._snapshot_err(started_at)
            if all(e is not None and e != DM_ERR_ENABLED for e in errs.values()):
                self.get_logger().info(
                    f'ch{self.channel}.id[{slot_str}] DISABLED '
                    f'({self._format_err_table(errs)})')
                return True
            time.sleep(0.05)
        # timeout
        errs = self._snapshot_err(started_at)
        bad = {s: e for s, e in errs.items() if e is None or e == DM_ERR_ENABLED}
        self.get_logger().warn(
            f'DISABLE timed out after {timeout:.1f}s; disable NOT confirmed: '
            f'{self._format_err_table(bad)}')
        return False


# --------------- interactive flow ---------------

def _live_print(prefix: str, value: Optional[float]) -> None:
    """Single-line live status. End the line later with a newline."""
    if value is None:
        sys.stdout.write(f'\r{prefix} (waiting for state...)        ')
    else:
        sys.stdout.write(f'\r{prefix} pos = {value:+.4f} rad        ')
    sys.stdout.flush()


def _wait_enter(prompt: str, stop_evt: threading.Event) -> bool:
    """Block on input() but allow Ctrl-C via stop_evt."""
    # input() itself raises KeyboardInterrupt under Ctrl-C, which the main
    # loop catches and turns into stop_evt.set(); the False return path
    # below is taken when the signal handler set stop_evt.
    sys.stdout.write(prompt)
    sys.stdout.flush()
    try:
        import select
        while not stop_evt.is_set():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                if sys.stdin.readline() == '':
                    stop_evt.set()
                break
    except (EOFError, KeyboardInterrupt):
        stop_evt.set()
        return False
    return not stop_evt.is_set()


def _read_key(stop_evt: threading.Event) -> Optional[str]:
    """Block until a single keypress. Returns:
        'enter'  -> ENTER / RETURN
        'space'  -> SPACEBAR
        None     -> Ctrl-C (stop_evt set) or stdin EOF
    Other keys are ignored. Uses cbreak mode so the live-display thread's
    \\n-terminated lines still scroll normally, but the user does not have
    to press ENTER to confirm a single-char choice. Restores tty on exit
    even on exception."""
    try:
        import termios
        import tty
        import select
    except ImportError:
        # Fall back to the line-buffered prompt: ENTER is accept, anything
        # else followed by ENTER is reject. Matches the spirit if the
        # platform has no tty (e.g. piped input in CI).
        line = ''
        try:
            line = input()
        except (EOFError, KeyboardInterrupt):
            stop_evt.set()
            return None
        return 'enter' if line.strip() == '' else 'space'

    fd = sys.stdin.fileno()
    try:
        old_attrs = termios.tcgetattr(fd)
    except termios.error:
        # Not a tty (piped/redirected); use line-buffered fallback.
        try:
            line = input()
        except (EOFError, KeyboardInterrupt):
            stop_evt.set()
            return None
        return 'enter' if line.strip() == '' else 'space'

    try:
        tty.setcbreak(fd)
        while not stop_evt.is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch == '':       # EOF
                stop_evt.set()
                return None
            if ch in ('\r', '\n'):
                return 'enter'
            if ch == ' ':
                return 'space'
            # Anything else: ignore and keep waiting.
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)


def _calibrate_zero_pose(
        node: CalibrationNode, cfg: CalibConfig,
        stop_evt: threading.Event) -> Optional[List[Dict[str, float]]]:
    """Single-shot zero-pose calibration: every joint at URDF zero, captured
    in one ENTER. Returns list of result dicts or None if aborted."""
    print()
    print('=' * 72)
    print('  ZERO-POSE CALIBRATION')
    print('  Place the whole arm in its URDF zero pose (use your fixture).')
    print(f'  All {len(cfg.joints)} joint(s) will be captured simultaneously.')
    print('=' * 72)

    live_stop = threading.Event()

    def _live_loop():
        # show every joint's live position on a single refreshing block
        n = len(cfg.joints)
        first = True
        while not live_stop.is_set() and not stop_evt.is_set():
            if not first:
                # move cursor up n lines to overwrite the previous block
                sys.stdout.write(f'\033[{n}A')
            for j in cfg.joints:
                s = node.get_state(j.slot)
                pos_str = (f'{s[0]:+.4f} rad' if s is not None
                           else '(no data)')
                # \033[K clears the rest of the line
                sys.stdout.write(
                    f'\r  [{j.name:<12s} slot {j.slot} {j.motor_type:<7s}] '
                    f'live = {pos_str}\033[K\n')
            sys.stdout.flush()
            first = False
            time.sleep(0.1)

    live_t = threading.Thread(target=_live_loop, daemon=True)
    live_t.start()

    while True:
        if not _wait_enter(
                '  >>> Place arm in URDF zero pose, hold steady, press '
                'ENTER (Ctrl-C aborts): ',
                stop_evt):
            live_stop.set()
            live_t.join(timeout=0.5)
            return None

        # capture window across all slots, atomically armed
        node.begin_sampling_all()
        time.sleep(cfg.window_sec)

        results: List[Dict[str, float]] = []
        bad_joints: List[str] = []
        too_few: List[str] = []
        for j in cfg.joints:
            samples = node.end_sampling(j.slot)
            if len(samples) < cfg.min_samples:
                too_few.append(f'{j.name}({len(samples)})')
                continue
            positions = [p for _, p in samples]
            spread = max(positions) - min(positions)
            median = statistics.median(positions)
            if spread > cfg.max_motion_rad:
                bad_joints.append(f'{j.name}(spread={spread:.4f})')
                continue
            zero_offset = median  # urdf_pos_at_reference = 0
            results.append({
                'slot': j.slot,
                'name': j.name,
                'motor_type': j.motor_type,
                'reference': 'urdf-zero-pose',
                'urdf_pos_at_reference': 0.0,
                'raw_at_reference': float(median),
                'sample_spread': float(spread),
                'sample_count': len(positions),
                'zero_offset': float(zero_offset),
                'axis_sign': j.axis_sign,
                'direction_verified': j.direction_verified,
            })

        if too_few:
            print(f'  ! not enough samples on: {", ".join(too_few)}; '
                  'is the bus alive? retry.')
            continue
        if bad_joints:
            print(f'  ! these joint(s) were moving (> {cfg.max_motion_rad} '
                  f'rad): {", ".join(bad_joints)}. Hold steady and retry.')
            continue

        live_stop.set()
        live_t.join(timeout=0.5)

        print()
        for r in results:
            print(f'  OK  {r["name"]:<12s} slot {r["slot"]} '
                  f'{r["motor_type"]:<7s}: raw={r["raw_at_reference"]:+.4f}'
                  f'   spread={r["sample_spread"]:.5f}'
                  f'   zero_offset={r["zero_offset"]:+.4f}')
        return results


def _calibrate_one(node: CalibrationNode, joint: JointSpec,
                   cfg: CalibConfig,
                   stop_evt: threading.Event) -> Optional[Dict[str, float]]:
    """Run interactive hard-stop calibration for one joint, with a verify
    phase: after capture, the live display switches to URDF-frame and the
    user can wiggle the joint to confirm sign/scale before accepting.
    Returns result dict or None if aborted."""
    print()
    print('=' * 72)
    print(f'  Joint: {joint.name}  (slot {joint.slot}, {joint.motor_type})')
    print(f'  Push to:     {joint.limit_side.upper()} hard stop')
    print(f'  URDF target: {joint.urdf_pos_at_limit:+.4f} rad  '
          f'({math.degrees(joint.urdf_pos_at_limit):+.2f} deg)')
    print('=' * 72)

    # Shared mutable state between the worker thread and the live display.
    # Phase: 'push' (show raw only, before capture) or
    #        'verify' (show urdf = raw - zero_offset, after capture).
    display = {'phase': 'push', 'zero_offset': 0.0}
    live_stop = threading.Event()

    def _live_loop():
        while not live_stop.is_set() and not stop_evt.is_set():
            s = node.get_state(joint.slot)
            if s is None:
                line = f'  [{joint.name}] live: (waiting for state...)'
            elif display['phase'] == 'push':
                line = f'  [{joint.name}] live: raw = {s[0]:+.4f} rad'
            else:
                urdf = joint.axis_sign * (s[0] - display['zero_offset'])
                line = (f'  [{joint.name}] verify: raw={s[0]:+.4f}  '
                        f'urdf={urdf:+.4f} rad  '
                        f'({math.degrees(urdf):+7.2f} deg)')
            # \033[2K erases the whole line; \r returns the cursor.
            sys.stdout.write(f'\r\033[2K{line}')
            sys.stdout.flush()
            time.sleep(0.1)
        # leave the cursor on a fresh line for whoever prints next
        sys.stdout.write('\n')
        sys.stdout.flush()

    live_t = threading.Thread(target=_live_loop, daemon=True)
    live_t.start()

    try:
        while True:
            # ---------- PHASE 1: push & capture ----------
            display['phase'] = 'push'
            if not _wait_enter(
                    '  >>> Push the joint into the hard stop and HOLD it; '
                    'press ENTER to capture (Ctrl-C aborts): ',
                    stop_evt):
                return None

            node.begin_sampling(joint.slot)
            time.sleep(cfg.window_sec)
            samples = node.end_sampling(joint.slot)

            if len(samples) < cfg.min_samples:
                print(f'  ! only got {len(samples)} samples '
                      f'(need >= {cfg.min_samples}); is the bus alive? retry.')
                continue

            positions = [p for _, p in samples]
            spread = max(positions) - min(positions)
            median = statistics.median(positions)

            if spread > cfg.max_motion_rad:
                print(f'  ! joint moved {spread:.4f} rad during the window '
                      f'(> {cfg.max_motion_rad}). Hold steady and retry.')
                continue

            zero_offset = reference_offset(median, joint.urdf_pos_at_limit, joint.axis_sign)

            print()
            print(f'  CAPTURED: raw_at_limit = {median:+.4f} rad  '
                  f'(spread {spread:.5f}, n={len(positions)})')
            print(f'            urdf_target  = {joint.urdf_pos_at_limit:+.4f} rad  '
                  f'({math.degrees(joint.urdf_pos_at_limit):+.2f} deg)')
            print(f'            zero_offset  = {zero_offset:+.4f} rad')
            print()
            print(f'  >>> VERIFY: still holding the limit, the live "urdf=" '
                  f'should read ~ {joint.urdf_pos_at_limit:+.4f} rad.')
            print(f'      Then release & wiggle the joint a little: urdf '
                  f'should change in the same direction as the joint moves')
            print(f'      and stay within URDF range. If it looks correct, '
                  f'press [ENTER] to accept and continue.')
            print(f'      If it looks wrong (sign/scale/wrap), press [SPACE] '
                  f'to redo this joint.')
            print()

            # ---------- PHASE 2: verify ----------
            display['zero_offset'] = zero_offset
            display['phase'] = 'verify'

            key = _read_key(stop_evt)
            if key is None:
                return None
            if key == 'space':
                print()
                print(f'  REJECTED. Back to push-and-capture for '
                      f'{joint.name}.')
                continue

            # accepted
            print()
            print(f'  ACCEPTED  {joint.name}  zero_offset = '
                  f'{zero_offset:+.4f} rad')
            return {
                'slot': joint.slot,
                'name': joint.name,
                'motor_type': joint.motor_type,
                'reference': f'hard-stop-{joint.limit_side}',
                'limit_side': joint.limit_side,
                'urdf_pos_at_reference': joint.urdf_pos_at_limit,
                'original_reference_angle': joint.reference_original,
                'reference_confirmed': joint.reference_confirmed,
                'raw_at_reference': float(median),
                'sample_spread': float(spread),
                'sample_count': len(positions),
                'zero_offset': float(zero_offset),
                'axis_sign': joint.axis_sign,
                'direction_verified': joint.direction_verified,
            }
    finally:
        live_stop.set()
        live_t.join(timeout=0.5)


def reference_offset(raw: float, reference: float, sign: int) -> float:
    if sign not in (-1, 1) or not all(math.isfinite(v) for v in (raw, reference)):
        raise ValueError('Invalid reference capture/sign')
    return raw - sign * reference


def _write_offsets_yaml(path: Path, channel: int, mode: str,
                        results: List[Dict[str, float]]) -> None:
    out = {
        'identified_at': datetime.datetime.now().isoformat(timespec='seconds'),
        'channel': channel,
        'mode': mode,
        'note': ('zero_offset = raw_at_reference - axis_sign * urdf_pos_at_reference. '
                 'Apply at runtime as urdf_pos = axis_sign * (raw_pos - '
                 'zero_offset). Do not change axis_sign without recomputing zero_offset.'),
        'offsets': results,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w') as f:
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=False)


# --------------- main ---------------

def review_limit_references(cfg, input_path, reviewed_path, stop_evt):
    """Run the visual gate using a plain ROS node, with no motor publishers."""
    if input_path.resolve() == reviewed_path.resolve():
        raise ValueError('Save reviewed references to a separate file, not the input YAML')
    node = Node('joint_limit_reference_review')
    preview = None
    try:
        preview = DirectionPreview(node, cfg, limit_review=True)
        preview.wait_ready(stop_evt)
        if not preview.review_limits(stop_evt):
            return False
        data = yaml.safe_load(input_path.read_text())
        by_name = {j.name: j for j in cfg.joints}
        for entry in data['joints']:
            joint = by_name[entry['name']]
            entry['urdf_pos_at_limit'] = joint.urdf_pos_at_limit
            entry['reference_review'] = {
                'original_angle_rad': joint.reference_original,
                'confirmed_angle_rad': joint.urdf_pos_at_limit,
                'confirmed': True}
        data['reference_reviewed_at'] = datetime.datetime.now().isoformat(timespec='seconds')
        reviewed_path.parent.mkdir(parents=True, exist_ok=True)
        reviewed_path.write_text(yaml.safe_dump(data, sort_keys=False))
        print(f'Reviewed mechanical reference angles saved to {reviewed_path}')
        return True
    finally:
        if preview is not None:
            preview.close()
        node.destroy_node()

def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Direction preview followed by manual reference calibration.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--mode', default='hard-stop',
                   choices=('hard-stop', 'zero-pose', 'direction', 'limit-preview'),
                   help='hard-stop: per-joint push to single-side limit. '
                        'zero-pose: arm placed at URDF zero by fixture, '
                        'one-shot capture of all joints. direction: MuJoCo incremental direction check. '
                        'limit-preview: edit and confirm reference angles without motor commands.')
    p.add_argument('--calibration-yaml', default='joint_calibration.yaml',
                   help='Input config; limit_side/urdf_pos_at_limit are '
                        'optional in zero-pose mode.')
    p.add_argument('--output', default='joint_offsets.yaml',
                   help='Where to write the resulting zero offsets.')
    p.add_argument('--directions-yaml', type=Path,
                   help='Verified direction file from --mode direction; used for reference capture.')
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    in_path = Path(args.calibration_yaml).expanduser().resolve()
    out_path = Path(args.output).expanduser().resolve()
    if args.mode == 'limit-preview' and args.output == 'joint_offsets.yaml':
        out_path = in_path.with_name(in_path.stem + '_reviewed.yaml')

    try:
        cfg = load_calibration_config(in_path)
        if args.mode == 'direction':
            if 'offset' in out_path.stem.lower():
                raise ValueError('Direction-only output must use a separate file, e.g. joint_directions_right.yaml')
        elif args.directions_yaml:
            load_directions(yaml.safe_load(args.directions_yaml.read_text()), cfg)
        elif args.mode != 'limit-preview':
            print('WARNING: no direction file; using unverified axis_sign=+1. '
                  'Use --mode direction first, then --directions-yaml.')
    except (FileNotFoundError, KeyError, ValueError) as e:
        print(f'Failed to load {in_path}: {e}', file=sys.stderr)
        return 2

    rclpy.init(args=argv)
    stop_evt = threading.Event()

    def _sig(*_):
        stop_evt.set()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    if args.mode in ('hard-stop', 'limit-preview'):
        reviewed_path = (out_path if args.mode == 'limit-preview'
                         else out_path.with_name(out_path.stem + '_references.yaml'))
        try:
            if not review_limit_references(cfg, in_path, reviewed_path, stop_evt):
                rclpy.shutdown()
                return 130
        except Exception as exc:
            print(f'Reference review failed: {exc}', file=sys.stderr)
            rclpy.shutdown()
            return 2
        if args.mode == 'limit-preview':
            rclpy.shutdown()
            return 0

    exec_ = MultiThreadedExecutor(num_threads=2)
    node = CalibrationNode(channel=cfg.channel, joints=cfg.joints)
    exec_.add_node(node)

    spin_stop = threading.Event()

    def _spin():
        try:
            # Keep feedback alive during the final disable handshake, including Ctrl-C.
            while rclpy.ok() and not spin_stop.is_set():
                exec_.spin_once(timeout_sec=0.1)
        except Exception as exc:  # noqa: BLE001
            node.get_logger().error(f'Calibration feedback executor failed: {exc}')
            stop_evt.set()
    spin_t = threading.Thread(target=_spin, daemon=True)
    spin_t.start()

    # NOTE: do NOT wait for state frames here -- DM motors only emit
    # feedback after they are enabled. The synchronous enable_all() below
    # publishes ENABLE at 5 Hz and waits until every motor reports a fresh
    # state on its own.

    # mode-specific input validation
    if args.mode == 'hard-stop':
        for j in cfg.joints:
            if j.limit_side == 'unused':
                print(f'hard-stop mode requires limit_side for slot {j.slot} '
                      f'({j.name}) in {in_path}', file=sys.stderr)
                stop_evt.set()
                rc_pre = 2
                break
        else:
            rc_pre = 0
        if stop_evt.is_set():
            try:
                spin_stop.set()
                spin_t.join(timeout=1.0)
                exec_.shutdown()
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
            if rclpy.ok():
                rclpy.shutdown()
            return rc_pre

    rc = 0
    results: List[Dict[str, float]] = []
    preview = None
    if not stop_evt.is_set():
        try:
            if args.mode == 'direction':
                # Window/model failures must happen before any ENABLE command.
                preview = DirectionPreview(node, cfg)
                preview.wait_ready(stop_evt)
            if not node.enable_all(timeout=5.0, stop_evt=stop_evt):
                # enable_all already logged the failure detail.
                stop_evt.set()
                rc = 5
                raise RuntimeError('enable failed; aborting calibration')
            print()
            print('All motors enabled in passive mode (kp=kd=tau_ff=0).')
            print('They will freewheel; support the arm before pushing.')

            if args.mode == 'direction':
                r = preview.run(stop_evt)
                if r is None:
                    rc = 130
                else:
                    results = r
            elif args.mode == 'zero-pose':
                r = _calibrate_zero_pose(node, cfg, stop_evt)
                if r is None:
                    print('Calibration aborted by user.')
                    rc = 130
                else:
                    results = r
            else:
                print('Hard-stop mode: calibration proceeds joint-by-joint '
                      'in YAML order.')
                for joint in cfg.joints:
                    if stop_evt.is_set():
                        break
                    r = _calibrate_one(node, joint, cfg, stop_evt)
                    if r is None:
                        print('Calibration aborted by user.')
                        rc = 130
                        break
                    results.append(r)
        except Exception as e:  # noqa: BLE001
            node.get_logger().error(f'Calibration failed: {e}')
            rc = 1

    # always disable. We do NOT pass stop_evt here -- a Ctrl-C should
    # still try to land the disable frames on the bus before exit.
    try:
        if not node.disable_all(timeout=3.0):
            rc = rc or 6
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'Disable NOT confirmed: {exc}')
        rc = rc or 6

    if results and rc == 0:
        try:
            if args.mode == 'direction':
                out_path.parent.mkdir(parents=True, exist_ok=True)
                out_path.write_text(yaml.safe_dump({
                    'mode': 'direction', 'channel': cfg.channel,
                    'identified_at': datetime.datetime.now().isoformat(timespec='seconds'),
                    'directions': results}, sort_keys=False))
            else:
                _write_offsets_yaml(out_path, cfg.channel, args.mode, results)
            print()
            print(f'Wrote {len(results)} {args.mode} calibration entries to {out_path}')
        except Exception as e:  # noqa: BLE001
            print(f'Failed to write {out_path}: {e}', file=sys.stderr)
            rc = 4

    if preview is not None:
        preview.close()
    try:
        spin_stop.set()
        spin_t.join(timeout=1.0)
        exec_.shutdown()
        node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    if rclpy.ok():
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
