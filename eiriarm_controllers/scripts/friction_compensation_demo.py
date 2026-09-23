#!/usr/bin/env python3
"""DM motor friction-compensation demo.

Loads a previously identified friction model from friction_model.yaml
and applies a feed-forward torque

    tau_ff(omega) = sign(omega) * coulomb(direction) + viscous * omega
                                                       (kinetic-only term)

so that, once the user pushes past breakaway, the motor feels (almost)
frictionless.  Static friction is NOT compensated -- the user must still
provide ~static torque to start motion.

Usage:

    # bring the bus up first (terminal A):
    ros2 launch eiriarm_bringup bridge.launch.py

    # single-motor mode (terminal B):
    ros2 run eiriarm_controllers friction_compensation_demo \\
        --motor-type DM4310 --channel 0 --slot 6 --gain 0.5

    # multi-motor mode -- compensate one full 7-DoF arm at once:
    ros2 run eiriarm_controllers friction_compensation_demo \\
        --channel 0 \\
        --motors 0:DM8009,1:DM8009,2:DM4340P,3:DM4340,4:DM4310,5:DM4310,6:DM4310 \\
        --gain 0.5

    # press any motor by hand; with gain >= ~1 it should feel transparent.
    # Ctrl-C to safely disable all motors.

Safety:
    * gain in [0.0 .. 1.5];  >1.0 risks self-excitation if coulomb is
      over-estimated (positive feedback through the velocity term).
    * deadband (default 0.05 rad/s) prevents jitter-driven creep.
    * tau_ff is hard-clamped to motor tor_max * 0.6.
    * Ctrl-C / SIGTERM both route through finally-block disable.
"""
import argparse
import dataclasses
import math
import signal
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


MOTOR_LIMITS: Dict[str, Tuple[float, float, float]] = {
    # type:    (pos_max, vel_max, tor_max)
    'DM4310':  (12.5, 50.0, 10.0),
    'DM4340':  (12.5, 20.0, 28.0),
    'DM4340P': (12.5, 20.0, 28.0),
    'DM8009':  (12.5, 45.0, 54.0),
}

NO_DATA_EPSILON = 1e-3

# DM motor state code (MotorState.error_flags = RX byte0 high 4 bits).
DM_ERR_ENABLED = 1
DM_ERR_NAMES = {
    0: 'disabled', 1: 'enabled',
    8: 'overvoltage', 9: 'undervoltage', 10: 'overcurrent',
    11: 'MOS-overtemp', 12: 'rotor-overtemp',
    13: 'comm-lost', 14: 'overload',
}


def _err_name(err):
    if err is None:
        return 'no-data'
    return DM_ERR_NAMES.get(err, f'unknown({err})')


@dataclasses.dataclass
class FrictionModel:
    coulomb_pos: float
    coulomb_neg: float
    viscous: float

    @classmethod
    def load(cls, yaml_path: Path, motor_type: str) -> 'FrictionModel':
        if not yaml_path.exists():
            raise FileNotFoundError(f'{yaml_path} not found')
        with yaml_path.open() as f:
            data = yaml.safe_load(f) or {}
        if motor_type not in data:
            raise KeyError(
                f'motor type "{motor_type}" not in {yaml_path}; '
                f'available: {sorted(data.keys())}')
        m = data[motor_type]
        for k in ('coulomb_pos', 'coulomb_neg', 'viscous'):
            if k not in m or m[k] is None or (isinstance(m[k], float)
                                              and math.isnan(m[k])):
                raise ValueError(
                    f'{yaml_path}::{motor_type}::{k} missing or NaN')
        return cls(coulomb_pos=float(m['coulomb_pos']),
                   coulomb_neg=float(m['coulomb_neg']),
                   viscous=float(m['viscous']))


@dataclasses.dataclass
class MotorEntry:
    slot: int
    motor_type: str
    pos_max: float
    vel_max: float
    tor_max: float
    model: FrictionModel
    tau_ff_clamp: float  # = max_ff_frac * tor_max


class FrictionCompensationNode(Node):
    """Compensate one or more DM motors on a single CAN channel."""

    def __init__(self, channel: int, motors: List[MotorEntry],
                 gain: float, deadband: float):
        super().__init__('friction_compensation_demo')
        if not motors:
            raise ValueError('motors list must not be empty')
        slots = [m.slot for m in motors]
        if len(set(slots)) != len(slots):
            raise ValueError(f'duplicate slot in motors: {slots}')

        self.channel = channel
        self.motors: List[MotorEntry] = motors
        self.gain = gain
        self.deadband = deadband

        self._cmd_pub = self.create_publisher(
            MotorCommandArray, '/w3_robot_bridge_node/commands', 10)
        self._state_sub = self.create_subscription(
            MotorStateArray, '/w3_robot_bridge_node/state',
            self._on_state, qos_profile_sensor_data)

        self._state_lock = threading.Lock()
        # slot -> (pos, vel, tor)
        self._states: Dict[int, Tuple[float, float, float]] = {}
        # slot -> latest MotorState.error_flags (only updated for non-sentinel frames)
        self._state_err: Dict[int, Optional[int]] = {m.slot: None for m in motors}
        # slot -> tau_ff last commanded
        self._tau_ffs: Dict[int, float] = {m.slot: 0.0 for m in motors}

        self._control_timer = self.create_timer(0.010, self._tick)
        self._log_timer = self.create_timer(1.0, self._tick_log)

        self.get_logger().info(
            f'Friction-compensation demo ready on ch{channel}: '
            f'gain={gain:.2f} deadband={deadband:.3f} rad/s, '
            f'tracking {len(motors)} motor(s)')
        for e in motors:
            self.get_logger().info(
                f'  slot {e.slot} [{e.motor_type}]: '
                f'coulomb=+{e.model.coulomb_pos:.4f}/-{e.model.coulomb_neg:.4f}'
                f'  viscous={e.model.viscous:.5f}'
                f'  ff_clamp=+-{e.tau_ff_clamp:.2f} N*m')

    # --------------- ROS callbacks ---------------

    def _on_state(self, msg: MotorStateArray) -> None:
        motors = {m.motor_index: m for m in msg.motors
                  if m.channel == self.channel and m.online
                  and all(math.isfinite(v) for v in (m.position, m.velocity, m.torque))}
        new_states: Dict[int, Tuple[float, float, float]] = {}
        new_errs: Dict[int, int] = {}
        for entry in self.motors:
            if entry.slot not in motors:
                continue
            m = motors[entry.slot]
            new_states[entry.slot] = (float(m.position),
                                      float(m.velocity),
                                      float(m.torque))
            new_errs[entry.slot] = int(m.error_flags)
        if not new_states:
            return
        with self._state_lock:
            self._states.update(new_states)
            self._state_err.update(new_errs)

    def _compute_ff(self, entry: MotorEntry, omega: float) -> float:
        if abs(omega) < self.deadband:
            return 0.0
        if omega > 0.0:
            tau = entry.model.coulomb_pos + entry.model.viscous * omega
        else:
            tau = -entry.model.coulomb_neg + entry.model.viscous * omega
        tau *= self.gain
        if tau > entry.tau_ff_clamp:
            tau = entry.tau_ff_clamp
        elif tau < -entry.tau_ff_clamp:
            tau = -entry.tau_ff_clamp
        return tau

    def _tick(self) -> None:
        # IMPORTANT: DM motors are passive in MIT mode -- they only emit
        # feedback after receiving a cmd. So we MUST publish every tick
        # (even before the first state frame), otherwise the bus stays
        # silent and we deadlock on "no state stream yet".
        with self._state_lock:
            states = dict(self._states)

        cmds: List[MotorCommand] = []
        for entry in self.motors:
            s = states.get(entry.slot)
            tau_ff = 0.0 if s is None else self._compute_ff(entry, s[1])
            self._tau_ffs[entry.slot] = tau_ff
            mc = MotorCommand()
            mc.channel = self.channel
            mc.mode = MotorCommand.MODE_RUN
            mc.motor_index = entry.slot
            mc.position = 0.0
            mc.velocity = 0.0
            mc.kp = 0.0
            mc.kd = 0.0
            mc.torque = float(tau_ff)
            cmds.append(mc)

        arr = MotorCommandArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.commands = cmds
        self._cmd_pub.publish(arr)

    def _tick_log(self) -> None:
        with self._state_lock:
            states = dict(self._states)
        if not states:
            self.get_logger().warn('No state stream yet on '
                                   '/w3_robot_bridge_node/state')
            return
        for entry in self.motors:
            s = states.get(entry.slot)
            if s is None:
                self.get_logger().info(
                    f'  slot {entry.slot} [{entry.motor_type}]: no data')
                continue
            pos, vel, tau_meas = s
            self.get_logger().info(
                f'  slot {entry.slot} [{entry.motor_type}]: '
                f'pos={pos:+.3f}  omega={vel:+.3f}  '
                f'tau_meas={tau_meas:+.3f}  '
                f'tau_ff={self._tau_ffs[entry.slot]:+.3f}')

    # --------------- enable / disable ---------------

    def _publish_enable_msg(self, enable: bool) -> None:
        msg = MotorCommandArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        mode = MotorCommand.MODE_ENABLE if enable else MotorCommand.MODE_DISABLE
        msg.commands = [
            MotorCommand(channel=self.channel, motor_index=slot, mode=mode)
            for slot in [entry.slot for entry in self.motors]
        ]
        self._cmd_pub.publish(msg)

    def _snapshot_err(self) -> Dict[int, Optional[int]]:
        with self._state_lock:
            return {e.slot: self._state_err.get(e.slot) for e in self.motors}

    def _format_err_table(self, errs: Dict[int, Optional[int]]) -> str:
        return ', '.join(
            f'slot{s}={_err_name(errs.get(s))}' for s in sorted(errs))

    def enable(self, timeout: float = 5.0,
               stop_evt: Optional[threading.Event] = None) -> bool:
        """Publish ENABLE at 5 Hz until every tracked motor reports
        MotorState.error_flags == DM_ERR_ENABLED, or the timeout expires.
        Returns True iff confirmed."""
        deadline = time.monotonic() + timeout
        last_pub = 0.0
        slot_str = ','.join(str(e.slot) for e in self.motors)
        self.get_logger().info(
            f'ch{self.channel}.id[{slot_str}] ENABLE: 5Hz publish until '
            f'every motor reports err=={DM_ERR_ENABLED} (timeout '
            f'{timeout:.1f}s)...')
        while time.monotonic() < deadline:
            if stop_evt is not None and stop_evt.is_set():
                return False
            now = time.monotonic()
            if now - last_pub >= 0.2:
                self._publish_enable_msg(True)
                last_pub = now
            errs = self._snapshot_err()
            if all(e == DM_ERR_ENABLED for e in errs.values()):
                self.get_logger().info(
                    f'ch{self.channel}.id[{slot_str}] ENABLED '
                    f'(all {len(self.motors)} motors report err=1)')
                return True
            time.sleep(0.02)
        errs = self._snapshot_err()
        bad = {s: e for s, e in errs.items() if e != DM_ERR_ENABLED}
        self.get_logger().error(
            f'ENABLE timed out after {timeout:.1f}s; '
            f'slot(s) not enabled: {self._format_err_table(bad)}')
        return False

    def disable(self, timeout: float = 3.0,
                stop_evt: Optional[threading.Event] = None) -> bool:
        """Publish DISABLE at 5 Hz until every tracked motor reports
        MotorState.error_flags != DM_ERR_ENABLED (disabled or error), or the
        timeout expires."""
        # Burst zero-cmd so motors are passive when DISABLE lands. Keep
        # the 10 ms cmd ticker running so the motors keep responding and
        # we can read fresh err codes (DM is request-response).
        arr = MotorCommandArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.commands = [
            MotorCommand(channel=self.channel, motor_index=e.slot, mode=MotorCommand.MODE_RUN, position=0.0, velocity=0.0,
                         kp=0.0, kd=0.0, torque=0.0)
            for e in self.motors
        ]
        for _ in range(3):
            self._cmd_pub.publish(arr)
            time.sleep(0.01)

        deadline = time.monotonic() + timeout
        last_pub = 0.0
        slot_str = ','.join(str(e.slot) for e in self.motors)
        self.get_logger().info(
            f'ch{self.channel}.id[{slot_str}] DISABLE: 5Hz publish until '
            f'every motor reports err!={DM_ERR_ENABLED} (timeout '
            f'{timeout:.1f}s)...')
        while time.monotonic() < deadline:
            if stop_evt is not None and stop_evt.is_set():
                self._publish_enable_msg(False)
                return False
            now = time.monotonic()
            if now - last_pub >= 0.2:
                self._publish_enable_msg(False)
                last_pub = now
            errs = self._snapshot_err()
            if all(e != DM_ERR_ENABLED for e in errs.values()):
                self.get_logger().info(
                    f'ch{self.channel}.id[{slot_str}] DISABLED '
                    f'({self._format_err_table(errs)})')
                return True
            time.sleep(0.05)
        errs = self._snapshot_err()
        bad = {s: e for s, e in errs.items() if e == DM_ERR_ENABLED}
        self.get_logger().warn(
            f'DISABLE timed out after {timeout:.1f}s; still enabled: '
            f'{self._format_err_table(bad)}')
        return False


def _parse_motors_arg(motors_str: str) -> List[Tuple[int, str]]:
    """Parse '0:DM8009,1:DM4340P,2:DM4340,3:DM4310' into [(slot, type)]."""
    out: List[Tuple[int, str]] = []
    for token in motors_str.split(','):
        token = token.strip()
        if not token:
            continue
        if ':' not in token:
            raise argparse.ArgumentTypeError(
                f'expected slot:type, got "{token}"')
        slot_s, mtype = token.split(':', 1)
        try:
            slot = int(slot_s)
        except ValueError:
            raise argparse.ArgumentTypeError(f'bad slot "{slot_s}"')
        if not 0 <= slot <= 7:
            raise argparse.ArgumentTypeError(
                f'slot must be 0..7, got {slot}')
        if mtype not in MOTOR_LIMITS:
            raise argparse.ArgumentTypeError(
                f'unknown motor type "{mtype}", '
                f'expected one of {sorted(MOTOR_LIMITS.keys())}')
        out.append((slot, mtype))
    if not out:
        raise argparse.ArgumentTypeError('--motors is empty')
    return out


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='DM motor friction-compensation demo (single or multi).',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--channel', type=int, default=0, choices=(0, 1))
    # multi-motor mode (preferred for whole arm)
    p.add_argument('--motors', type=_parse_motors_arg, default=None,
                   help='Comma-separated slot:type list, e.g. '
                        '"0:DM8009,1:DM4340P,2:DM4340,3:DM4310". '
                        'Overrides --motor-type/--slot.')
    # legacy single-motor mode
    p.add_argument('--motor-type', default=None,
                   choices=sorted(MOTOR_LIMITS.keys()),
                   help='single-motor mode (used only when --motors absent)')
    p.add_argument('--slot', type=int, default=0,
                   help='single-motor mode slot (used only when --motors '
                        'absent)')
    p.add_argument('--gain', type=float, default=0.5,
                   help='compensation gain (0=off, 1=full model). Stay <=1.2.')
    p.add_argument('--deadband', type=float, default=0.05,
                   help='|omega| below this -> no feed-forward [rad/s]')
    p.add_argument('--max-ff-frac', type=float, default=0.6,
                   help='clamp |tau_ff| to this fraction of tor_max')
    p.add_argument('--model-yaml', default='friction_model.yaml')
    args = p.parse_args(argv)
    if args.motors is None and args.motor_type is None:
        p.error('must provide either --motors or --motor-type')
    return args


def _build_motor_entries(
        motors: List[Tuple[int, str]], yaml_path: Path,
        max_ff_frac: float) -> List[MotorEntry]:
    out: List[MotorEntry] = []
    for slot, mtype in motors:
        pos_max, vel_max, tor_max = MOTOR_LIMITS[mtype]
        model = FrictionModel.load(yaml_path, mtype)
        out.append(MotorEntry(
            slot=slot, motor_type=mtype,
            pos_max=pos_max, vel_max=vel_max, tor_max=tor_max,
            model=model,
            tau_ff_clamp=max_ff_frac * tor_max))
    return out


def main(argv=None) -> int:
    args = parse_args(argv)
    if not (0.0 <= args.gain <= 1.5):
        print(f'gain must be in [0, 1.5], got {args.gain}', file=sys.stderr)
        return 2
    if args.gain > 1.2:
        print(f'WARNING: gain={args.gain:.2f} > 1.2, risk of self-excitation',
              file=sys.stderr)

    motors_list: List[Tuple[int, str]] = (
        args.motors if args.motors is not None
        else [(args.slot, args.motor_type)])

    yaml_path = Path(args.model_yaml).expanduser().resolve()
    try:
        entries = _build_motor_entries(
            motors_list, yaml_path, args.max_ff_frac)
    except (FileNotFoundError, KeyError, ValueError) as e:
        print(f'Failed to load friction model: {e}', file=sys.stderr)
        return 3

    rclpy.init(args=argv)
    exec_ = MultiThreadedExecutor(num_threads=2)
    node = FrictionCompensationNode(
        channel=args.channel, motors=entries,
        gain=args.gain, deadband=args.deadband)
    exec_.add_node(node)

    # graceful Ctrl-C: stop spinning, then disable in main thread.
    stop_evt = threading.Event()

    def _sig(*_):
        stop_evt.set()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    def _spin():
        try:
            while rclpy.ok() and not stop_evt.is_set():
                exec_.spin_once(timeout_sec=0.1)
        except Exception:  # noqa: BLE001
            stop_evt.set()
    t = threading.Thread(target=_spin, daemon=True)
    t.start()

    # NOTE: do NOT wait for state here -- DM motors only stream after
    # being enabled. enable() below publishes ENABLE at 5 Hz and waits
    # until every motor reports a fresh state on its own.
    if not stop_evt.is_set():
        if not node.enable(timeout=5.0, stop_evt=stop_evt):
            stop_evt.set()
        else:
            node.get_logger().info(
                'Compensation active. Push any motor by hand. Ctrl-C to stop.')

    try:
        while not stop_evt.is_set():
            time.sleep(0.1)
    finally:
        # No stop_evt here -- Ctrl-C should still try to land disable frames.
        try:
            node.disable(timeout=3.0)
        except Exception:  # noqa: BLE001
            pass
        try:
            exec_.shutdown()
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
