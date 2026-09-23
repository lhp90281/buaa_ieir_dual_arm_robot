#!/usr/bin/env python3
"""DM motor friction identification via USB2CAN ROS topics.

Identifies Coulomb + viscous + static friction for a single DM motor
(directly driven, no gearbox).

Algorithm port of qt5_damiao_motor_friction_detection (Chen XingYu),
with: GUI/QThread/DM_CAN.py removed; data acquisition via USB2CAN
/w3_robot_bridge_node/state; joint identification of (Coulomb, viscous) via
numpy.linalg.lstsq instead of asking the user to pre-fill viscous_coeff.

Usage example:

    # terminal A: bring up the bus
    ros2 launch eiriarm_bringup bridge.launch.py

    # terminal B: run identification on one motor
    ros2 run eiriarm_controllers friction_identification \\
        --motor-type DM8009 --channel 0 --slot 0 \\
        --tests baseline,coulomb,static

Outputs:
    friction_model.yaml  (merged per motor type)
    friction_results/<motor_type>_<test>_<timestamp>.png
"""
import argparse
import math
import dataclasses
import datetime as _dt
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402

from w3_robot_bridge.msg import (  # noqa: E402
    MotorCommand,
    MotorCommandArray,
    MotorStateArray,
)

# Known DM motor flash *_max values (from real-hardware-bringup plan §0).
MOTOR_LIMITS: Dict[str, Tuple[float, float]] = {
    # type:   (vel_max [rad/s], tor_max [N*m])
    'DM4310':  (50.0, 10.0),
    'DM4340':  (20.0, 28.0),
    'DM4340P': (20.0, 28.0),
    'DM8009':  (45.0, 54.0),
}


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
class MotorCmd:
    kp: float = 0.0
    kd: float = 0.0
    position: float = 0.0
    velocity: float = 0.0
    torque_ff: float = 0.0


class FrictionIdentifierNode(Node):
    """ROS I/O: 100 Hz cmd heartbeat, state sub, enable/disable helpers."""

    def __init__(self, channel: int, slot: int, motor_type: str,
                 pos_max: float, vel_max: float, tor_max: float):
        super().__init__('friction_identification')
        if channel not in (0, 1):
            raise ValueError(f'channel must be 0/1, got {channel}')
        if not 0 <= slot <= 7:
            raise ValueError(f'slot must be 0..7, got {slot}')
        self.channel = channel
        self.slot = slot
        self.motor_type = motor_type
        self.pos_max = pos_max
        self.vel_max = vel_max
        self.tor_max = tor_max

        self._cmd_pub = self.create_publisher(
            MotorCommandArray, '/w3_robot_bridge_node/commands', 10)
        self._state_sub = self.create_subscription(
            MotorStateArray, '/w3_robot_bridge_node/state',
            self._on_state, qos_profile_sensor_data)

        self._cmd = MotorCmd()
        self._cmd_lock = threading.Lock()
        self._heartbeat_timer = self.create_timer(0.010, self._publish_cmd)

        self._state_lock = threading.Lock()
        self._latest: Optional[Tuple[float, float, float, float]] = None
        # Latest MotorState.error_flags for this motor (None if no real frame yet).
        self._latest_err: Optional[int] = None

        self.get_logger().info(
            f'Friction identifier ready: ch{channel}.id{slot} [{motor_type}] '
            f'limits pos+-{pos_max} vel+-{vel_max} tor+-{tor_max}')

    def _on_state(self, msg: MotorStateArray) -> None:
        m = next((m for m in msg.motors
                  if m.channel == self.channel and m.motor_index == self.slot
                  and m.online), None)
        if m is None or not all(math.isfinite(v) for v in (m.position, m.velocity, m.torque)):
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._state_lock:
            self._latest = (t, float(m.position),
                            float(m.velocity), float(m.torque))
            self._latest_err = int(m.error_flags)

    def _publish_cmd(self) -> None:
        with self._cmd_lock:
            c = dataclasses.replace(self._cmd)
        arr = MotorCommandArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        mc = MotorCommand()
        mc.channel = self.channel
        mc.mode = MotorCommand.MODE_RUN
        mc.motor_index = self.slot
        mc.position = float(c.position)
        mc.velocity = float(c.velocity)
        mc.kp = float(c.kp)
        mc.kd = float(c.kd)
        mc.torque = float(c.torque_ff)
        arr.commands = [mc]
        self._cmd_pub.publish(arr)

    def set_cmd(self, kp: float = 0.0, kd: float = 0.0,
                position: float = 0.0, velocity: float = 0.0,
                torque_ff: float = 0.0) -> None:
        with self._cmd_lock:
            self._cmd = MotorCmd(kp=kp, kd=kd, position=position,
                                 velocity=velocity, torque_ff=torque_ff)

    def zero_cmd(self) -> None:
        self.set_cmd()

    def get_state(self, timeout: float = 1.0
                  ) -> Optional[Tuple[float, float, float, float]]:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self._state_lock:
                s = self._latest
            if s is not None:
                return s
            time.sleep(0.005)
        return None

    def _publish_enable_msg(self, enable: bool) -> None:
        msg = MotorCommandArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        mode = MotorCommand.MODE_ENABLE if enable else MotorCommand.MODE_DISABLE
        msg.commands = [
            MotorCommand(channel=self.channel, motor_index=slot, mode=mode)
            for slot in [self.slot]
        ]
        self._cmd_pub.publish(msg)

    def _get_latest_err(self) -> Optional[int]:
        with self._state_lock:
            return self._latest_err

    def enable(self, timeout: float = 5.0) -> bool:
        """Publish ENABLE at 5 Hz until MotorState.error_flags == DM_ERR_ENABLED,
        or the timeout expires. Returns True iff confirmed."""
        deadline = time.monotonic() + timeout
        last_pub = 0.0
        self.get_logger().info(
            f'ch{self.channel}.id{self.slot} ENABLE: 5Hz publish until motor '
            f'reports err=={DM_ERR_ENABLED} (timeout {timeout:.1f}s)...')
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_pub >= 0.2:
                self._publish_enable_msg(True)
                last_pub = now
            if self._get_latest_err() == DM_ERR_ENABLED:
                self.get_logger().info(
                    f'ch{self.channel}.id{self.slot} ENABLED (err=1)')
                return True
            time.sleep(0.02)
        err = self._get_latest_err()
        self.get_logger().error(
            f'ENABLE timed out after {timeout:.1f}s; motor state: '
            f'{_err_name(err)}')
        return False

    def disable(self, timeout: float = 3.0) -> bool:
        """Publish DISABLE at 5 Hz until MotorState.error_flags != DM_ERR_ENABLED
        (disabled or in an error state), or the timeout expires."""
        # Zero the cmd so the motor is passive when DISABLE lands. Keep the
        # cmd heartbeat running -- the motor is request-response, so without
        # it we'd never see a fresh err code.
        self.zero_cmd()
        time.sleep(0.05)

        deadline = time.monotonic() + timeout
        last_pub = 0.0
        self.get_logger().info(
            f'ch{self.channel}.id{self.slot} DISABLE: 5Hz publish until motor '
            f'reports err!={DM_ERR_ENABLED} (timeout {timeout:.1f}s)...')
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_pub >= 0.2:
                self._publish_enable_msg(False)
                last_pub = now
            err = self._get_latest_err()
            if err is not None and err != DM_ERR_ENABLED:
                self.get_logger().info(
                    f'ch{self.channel}.id{self.slot} DISABLED '
                    f'({_err_name(err)})')
                return True
            time.sleep(0.05)
        err = self._get_latest_err()
        self.get_logger().warn(
            f'DISABLE timed out after {timeout:.1f}s; motor state still: '
            f'{_err_name(err)}')
        return False


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='DM motor friction identification via USB2CAN.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--motor-type', required=True,
                   choices=sorted(MOTOR_LIMITS.keys()))
    p.add_argument('--channel', type=int, default=0, choices=(0, 1))
    p.add_argument('--slot', type=int, default=0)
    p.add_argument('--tests', default='baseline,coulomb,static',
                   help='comma-separated subset of {baseline,coulomb,static}')
    p.add_argument('--vel-cap', type=float, default=0.5,
                   help='velocity sweep fraction of motor vel_max')
    p.add_argument('--tor-cap', type=float, default=0.5,
                   help='static ramp fraction of motor tor_max')
    p.add_argument('--kv', type=float, default=0.5,
                   help='velocity-loop kd (kp=0 always)')
    p.add_argument('--settling-time', type=float, default=2.0)
    p.add_argument('--sample-duration', type=float, default=3.0)
    p.add_argument('--baseline-duration', type=float, default=15.0)
    p.add_argument('--baseline-thresh', type=float, default=0.1)
    p.add_argument('--movement-threshold', type=float, default=0.05,
                   help='static breakaway |omega| threshold [rad/s]')
    p.add_argument('--pos-max', type=float, default=12.5)
    p.add_argument('--output-yaml', default='friction_model.yaml')
    p.add_argument('--output-dir', default='friction_results')
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    tests = [t.strip() for t in args.tests.split(',') if t.strip()]
    unknown = set(tests) - {'baseline', 'coulomb', 'static'}
    if unknown:
        print(f'Unknown tests: {unknown}', file=sys.stderr)
        return 2

    vel_max, tor_max = MOTOR_LIMITS[args.motor_type]
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    output_yaml = Path(args.output_yaml).expanduser().resolve()

    rclpy.init()
    node = FrictionIdentifierNode(
        channel=args.channel, slot=args.slot,
        motor_type=args.motor_type,
        pos_max=args.pos_max, vel_max=vel_max, tor_max=tor_max)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    results_all: Dict[str, float] = {}
    exit_code = 0
    try:
        node.get_logger().info('Waiting for first /w3_robot_bridge_node/state frame...')
        if node.get_state(timeout=5.0) is None:
            raise RuntimeError(
                'No /w3_robot_bridge_node/state received within 5 s. '
                'Is bridge.launch.py launch running?')
        node.get_logger().info('State stream OK, starting tests.')

        if 'baseline' in tests:
            mean, std = baseline_check(
                node, duration=args.baseline_duration,
                thresh=args.baseline_thresh)
            results_all['baseline_torque_mean'] = mean
            results_all['baseline_torque_std'] = std

        if 'coulomb' in tests:
            r = identify_coulomb_viscous(
                node, vel_cap=args.vel_cap, kv=args.kv,
                settling_time=args.settling_time,
                sample_duration=args.sample_duration,
                output_dir=output_dir)
            results_all.update({k: v for k, v in r.items()
                                if k != 'per_setpoint'})

        if 'static' in tests:
            r = identify_static(
                node, tor_cap=args.tor_cap,
                movement_threshold=args.movement_threshold,
                output_dir=output_dir)
            results_all.update(r)

        slot_str = f'ch{args.channel}.id{args.slot}'
        merge_friction_yaml(output_yaml, args.motor_type,
                            slot_str, results_all)

        print('\n========== Friction identification done ==========')
        for k, v in results_all.items():
            print(f'  {k:24s}: {v}')

    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted by user.')
        exit_code = 130
    except Exception:
        node.get_logger().error('Friction identification failed:')
        traceback.print_exc()
        exit_code = 1
    finally:
        try:
            node.zero_cmd()
            time.sleep(0.05)
            node.disable()
        except Exception:
            pass
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


# ---------------------------------------------------------------------------
# Algorithms
# ---------------------------------------------------------------------------

def baseline_check(node: FrictionIdentifierNode, duration: float = 15.0,
                   thresh: float = 0.1) -> Tuple[float, float]:
    """Capture motor-disabled torque feedback for `duration` s.

    If |mean| > thresh, warn that the shaft is probably tilted (gravity
    bleeding into the torque channel) or the fixture is preloaded; either
    biases the later friction estimates.
    """
    log = node.get_logger().info
    log(f'Baseline check: capturing {duration:.1f}s of torque '
        'with motor disabled...')
    node.disable()
    time.sleep(0.5)

    torques: List[float] = []
    velocities: List[float] = []
    t0 = time.monotonic()
    last_print = t0
    while time.monotonic() - t0 < duration:
        s = node.get_state(timeout=0.1)
        if s is not None:
            _, _, vel, tor = s
            torques.append(tor)
            velocities.append(vel)
        now = time.monotonic()
        if now - last_print >= 5.0:
            elapsed = now - t0
            log(f'  baseline progress {elapsed:.1f}/{duration:.1f}s '
                f'samples={len(torques)}')
            last_print = now
        time.sleep(0.01)

    if len(torques) < 100:
        raise RuntimeError(
            f'baseline: too few samples ({len(torques)}). '
            'Check /w3_robot_bridge_node/state is publishing.')

    arr = np.asarray(torques)
    vel_arr = np.asarray(velocities)
    mean = float(arr.mean())
    std = float(arr.std())
    vmean = float(vel_arr.mean())
    vstd = float(vel_arr.std())

    log(f'Baseline torque:   mean={mean:+.4f} N*m, std={std:.4f}')
    log(f'Baseline velocity: mean={vmean:+.4f} rad/s, std={vstd:.4f}')

    if abs(mean) > thresh:
        node.get_logger().warn(
            f'Baseline |mean| {abs(mean):.3f} > {thresh:.3f} N*m. '
            'Motor shaft is probably not vertical, or fixture has stress. '
            'Friction estimates will be biased; consider re-mounting.')
    else:
        log(f'Baseline OK (|mean| < {thresh:.3f} N*m).')
    return mean, std


def identify_coulomb_viscous(node: FrictionIdentifierNode,
                             vel_cap: float, kv: float,
                             settling_time: float, sample_duration: float,
                             output_dir: Path) -> Dict[str, object]:
    """Sweep velocities, joint-fit tau = sign(w)*tau_c + b*w via lstsq.

    Velocity control mode: (kp=0, kd=kv, vel=target). After `settling_time`
    s, collect `sample_duration` s of (vel, tor) and take the mean.
    Then fit the friction model on |omega| >= 0.3 rad/s window only
    (low-speed = stiction noise zone, excluded).
    """
    log = node.get_logger().info
    vmax = node.vel_max * vel_cap
    setpoints_norm = [-1.0, -0.7, -0.4, -0.2, -0.1,
                      0.1, 0.2, 0.4, 0.7, 1.0]
    setpoints = [v * vmax for v in setpoints_norm]

    log(f'Coulomb+viscous sweep: vmax={vmax:.2f} rad/s '
        f'(vel_cap={vel_cap:.2f} of vel_max={node.vel_max}), '
        f'kv={kv}, settling={settling_time}s, duration={sample_duration}s')

    node.enable()
    samples_vel: List[float] = []
    samples_tor: List[float] = []
    per_setpoint: List[Dict[str, float]] = []

    try:
        for i, target in enumerate(setpoints):
            log(f'  [{i+1}/{len(setpoints)}] target {target:+.3f} rad/s, '
                'settling...')
            node.set_cmd(kp=0.0, kd=kv, position=0.0,
                         velocity=target, torque_ff=0.0)
            time.sleep(settling_time)

            t0 = time.monotonic()
            vs: List[float] = []
            ts: List[float] = []
            while time.monotonic() - t0 < sample_duration:
                s = node.get_state(timeout=0.05)
                if s is not None:
                    _, _, v, t = s
                    vs.append(v)
                    ts.append(t)
                time.sleep(0.005)
            if len(vs) < 20:
                node.get_logger().warn(
                    f'  setpoint {target:+.2f} got only {len(vs)} samples, '
                    'skipping')
                continue
            v_mean = float(np.mean(vs))
            t_mean = float(np.mean(ts))
            t_std = float(np.std(ts))
            log(f'  setpoint {target:+.3f}: '
                f'actual w={v_mean:+.3f} tau={t_mean:+.4f} +- {t_std:.4f}')
            per_setpoint.append({
                'target_velocity': target,
                'actual_velocity': v_mean,
                'mean_torque': t_mean,
                'std_torque': t_std,
                'n_samples': len(vs),
            })
            samples_vel.append(v_mean)
            samples_tor.append(t_mean)
    finally:
        node.zero_cmd()
        time.sleep(0.05)
        node.disable()

    omega = np.asarray(samples_vel)
    tau = np.asarray(samples_tor)
    mask = np.abs(omega) >= 0.3
    if int(mask.sum()) < 4:
        raise RuntimeError(
            f'Not enough high-speed points for lstsq fit '
            f'(got {int(mask.sum())}, need >= 4). Try larger --vel-cap.')
    om = omega[mask]
    ta = tau[mask]
    pos_ind = (om > 0).astype(float)
    neg_ind = (om < 0).astype(float)
    A = np.column_stack([pos_ind, neg_ind, om])
    sol, _, _, _ = np.linalg.lstsq(A, ta, rcond=None)
    a_pos, a_neg, b = float(sol[0]), float(sol[1]), float(sol[2])
    coulomb_pos = a_pos
    coulomb_neg = -a_neg
    viscous = b
    log(f'Coulomb fit:  tau_c+ = {coulomb_pos:+.4f} N*m, '
        f'tau_c- = {coulomb_neg:+.4f} N*m')
    log(f'Viscous:      b = {viscous:+.5f} N*m*s/rad')

    _plot_coulomb(samples_vel, samples_tor, coulomb_pos, coulomb_neg,
                  viscous, node.motor_type, output_dir)

    return {
        'coulomb_pos': coulomb_pos,
        'coulomb_neg': coulomb_neg,
        'viscous': viscous,
        'per_setpoint': per_setpoint,
    }


def identify_static(node: FrictionIdentifierNode,
                    tor_cap: float, movement_threshold: float,
                    output_dir: Path) -> Dict[str, float]:
    """Ramp torque, record breakaway when |omega| first exceeds threshold."""
    log = node.get_logger().info
    tor_max = node.tor_max * tor_cap
    inc = 0.005 * (node.tor_max / 10.0)
    log(f'Static ramp: cap=tor_max*{tor_cap:.2f}={tor_max:.2f} N*m, '
        f'inc={inc:.4f} N*m/10ms, breakaway threshold {movement_threshold} '
        'rad/s')

    breakaways: Dict[str, float] = {}
    trajectories: Dict[str,
                       Tuple[List[float], List[float], List[float]]] = {}
    node.enable()

    try:
        for direction in (+1, -1):
            label = 'pos' if direction > 0 else 'neg'
            log(f'  direction {label}: torque ramp...')
            node.zero_cmd()
            time.sleep(1.5)

            ff = 0.0
            t_arr: List[float] = []
            v_arr: List[float] = []
            tor_arr: List[float] = []
            t0 = time.monotonic()
            broken: Optional[float] = None
            while ff < tor_max:
                node.set_cmd(kp=0.0, kd=0.0, position=0.0,
                             velocity=0.0, torque_ff=direction * ff)
                time.sleep(0.01)
                s = node.get_state(timeout=0.05)
                if s is None:
                    continue
                _, _, vel, _ = s
                t_arr.append(time.monotonic() - t0)
                v_arr.append(vel)
                tor_arr.append(ff)
                if abs(vel) > movement_threshold:
                    broken = ff
                    log(f'    breakaway at ff={ff:.4f} N*m, '
                        f'w={vel:+.4f} rad/s')
                    break
                ff += inc

            node.zero_cmd()
            time.sleep(0.5)

            if broken is None:
                node.get_logger().warn(
                    f'  direction {label}: reached cap {tor_max:.2f} N*m '
                    'without breakaway. Try larger --tor-cap.')
                breakaways[label] = float('nan')
            else:
                breakaways[label] = broken
            trajectories[label] = (t_arr, tor_arr, v_arr)
    finally:
        node.zero_cmd()
        time.sleep(0.05)
        node.disable()

    _plot_static(trajectories, breakaways, node.motor_type, output_dir)

    return {
        'static_pos': breakaways.get('pos', float('nan')),
        'static_neg': breakaways.get('neg', float('nan')),
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def _plot_coulomb(speeds: List[float], torques: List[float],
                  coulomb_pos: float, coulomb_neg: float,
                  viscous: float, motor_type: str, output_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.scatter(speeds, torques, s=60, c='steelblue', alpha=0.8,
               label='measured (omega, torque)')
    if speeds:
        lo = min(speeds) * 1.05
        hi = max(speeds) * 1.05
    else:
        lo, hi = -1.0, 1.0
    x = np.linspace(lo, hi, 400)
    y = np.where(x > 0, coulomb_pos + viscous * x,
                 -coulomb_neg + viscous * x)
    ax.plot(x, y, 'r-', linewidth=2, label='fit: sign(w)*tau_c + b*w')
    ax.axhline(coulomb_pos, color='g', linestyle='--', alpha=0.6,
               label=f'tau_c+ = {coulomb_pos:+.4f}')
    ax.axhline(-coulomb_neg, color='m', linestyle='--', alpha=0.6,
               label=f'-tau_c- = {-coulomb_neg:+.4f}')
    ax.axhline(0, color='k', linewidth=0.5, alpha=0.4)
    ax.axvline(0, color='k', linewidth=0.5, alpha=0.4)
    ax.set_xlabel('Angular velocity omega [rad/s]')
    ax.set_ylabel('Steady-state feedback torque [N*m]')
    ax.set_title(f'{motor_type} - Coulomb + viscous identification')
    ax.legend(loc='best', fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    ts = _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    path = output_dir / f'{motor_type}_coulomb_{ts}.png'
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'Saved {path}')


def _plot_static(traj: Dict[str,
                            Tuple[List[float], List[float], List[float]]],
                 breakaways: Dict[str, float],
                 motor_type: str, output_dir: Path) -> None:
    n = len(traj)
    if n == 0:
        return
    fig, axes = plt.subplots(n, 2, figsize=(12, 4 * n), squeeze=False)
    for i, (label, (t, ff, vel)) in enumerate(traj.items()):
        ax_tor = axes[i][0]
        ax_vel = axes[i][1]
        ax_tor.plot(t, ff, 'r-', linewidth=1.5, label='applied ff')
        b = breakaways.get(label, float('nan'))
        if not np.isnan(b):
            ax_tor.axhline(b, color='g', linestyle='--',
                           label=f'breakaway = {b:.4f}')
        ax_tor.legend(fontsize=9)
        ax_tor.set_ylabel('Applied torque [N*m]')
        ax_tor.set_title(f'{motor_type} - static ramp ({label})')
        ax_tor.grid(alpha=0.3)
        ax_vel.plot(t, vel, 'g-', linewidth=1.5)
        ax_vel.axhline(0, color='k', linewidth=0.5, alpha=0.4)
        ax_vel.set_xlabel('Time [s]')
        ax_vel.set_ylabel('Angular velocity [rad/s]')
        ax_vel.grid(alpha=0.3)
    fig.tight_layout()
    ts = _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    path = output_dir / f'{motor_type}_static_{ts}.png'
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'Saved {path}')


# ---------------------------------------------------------------------------
# YAML I/O
# ---------------------------------------------------------------------------

def merge_friction_yaml(yaml_path: Path, motor_type: str, slot_str: str,
                        results: Dict[str, float]) -> None:
    if yaml_path.exists():
        with yaml_path.open('r') as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {}
    entry = data.get(motor_type, {})
    entry.update({
        'viscous':     results.get('viscous'),
        'coulomb_pos': results.get('coulomb_pos'),
        'coulomb_neg': results.get('coulomb_neg'),
        'static_pos':  results.get('static_pos'),
        'static_neg':  results.get('static_neg'),
        'identified_at': _dt.datetime.now().isoformat(timespec='seconds'),
        'identified_on_slot': slot_str,
    })
    entry = {k: v for k, v in entry.items() if v is not None}
    data[motor_type] = entry
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    with yaml_path.open('w') as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)
    print(f'Updated {yaml_path} (entry "{motor_type}")')


if __name__ == '__main__':
    sys.exit(main())
