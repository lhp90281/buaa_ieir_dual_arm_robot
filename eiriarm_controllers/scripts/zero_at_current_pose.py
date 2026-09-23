#!/usr/bin/env python3
"""Set zero_offset from the current robot pose.

Drag-to-zero calibration:
    Stand the robot at URDF zero pose (visually align every joint against
    the URDF default pose, e.g. via RViz robot_state_publisher), then run
    this script. It reads the latest raw motor reading of every joint
    listed in the offsets YAML, averages a few frames, and writes
        zero_offset := raw_at_zero
    so that at this pose
        urdf_pos = axis_sign * (raw - zero_offset) = 0
    holds regardless of axis_sign sign.

This avoids the indirection of hard-stop calibration (where you must know
the URDF angle of the limit you happened to push against). It does NOT
fix axis_sign mistakes -- if a motor's positive direction is opposite
the URDF axis, the zero pose will be correct but motion direction will
still be inverted; verify and edit axis_sign manually after running this.

Pre-conditions:
    - W3 bridge is running (publishing /w3_robot_bridge_node/state)
    - every motor in the YAML is enabled (so it keeps replying state frames)
    - the robot is physically at URDF zero pose (and stationary)

Usage:
    ros2 run eiriarm_controllers zero_at_current_pose \
         --offsets joint_offsets_dual.yaml          # dry-run, shows diff
    ros2 run eiriarm_controllers zero_at_current_pose \
         --offsets joint_offsets_dual.yaml --apply  # writes file
"""
import argparse
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

MOTOR_LIMITS = {
    'DM4310': (12.5, 50.0, 10.0),
    'DM4340': (12.5, 20.0, 28.0),
    'DM4340P': (12.5, 20.0, 28.0),
    'DM8009': (12.5, 45.0, 54.0),
}

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from w3_robot_bridge.msg import MotorStateArray


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Set zero_offset = current raw, for drag-to-zero calibration.')
    p.add_argument('--offsets', type=Path, required=True,
                   help='Path to offsets YAML (read and rewritten in place).')
    p.add_argument('--apply', action='store_true',
                   help='Actually write the file. Without this, dry-run only.')
    p.add_argument('--samples', type=int, default=50,
                   help='Number of state frames to average per motor (default 50).')
    p.add_argument('--timeout', type=float, default=15.0,
                   help='Total timeout in seconds to gather samples (default 15).')
    p.add_argument('--max-spread', type=float, default=0.02,
                   help='Reject samples if peak-to-peak spread exceeds this (rad). '
                        'Default 0.02 rad ~= 1.1 deg. Hold the robot still.')
    p.add_argument('--motor-type-map', type=Path, default=None,
                   help='Optional YAML with per-slot motor_type entries.')
    return p.parse_args(argv)


class RawCollector(Node):
    """Subscribes to /w3_robot_bridge_node/state on every requested channel and stores
    the latest MotorStateArray frame per channel. We dedupe by frame identity
    so only fresh frames are counted toward `samples`."""

    def __init__(self, channels: List[int]):
        super().__init__('zero_at_current_pose')
        self._latest: Dict[int, Optional[MotorStateArray]] = {ch: None for ch in channels}
        self._last_id: Dict[int, Optional[int]] = {ch: None for ch in channels}
        self._sub = self.create_subscription(
            MotorStateArray, '/w3_robot_bridge_node/state',
            self._on_state, qos_profile_sensor_data)
        self.get_logger().info('Subscribed to /w3_robot_bridge_node/state')

    def _on_state(self, msg: MotorStateArray):
        for ch in self._latest:
            self._latest[ch] = msg

    def gather(self, channels: List[int], samples: int,
               timeout: float,
               limits: Dict[Tuple[int, int], Tuple[float, float, float]]) -> Tuple[Dict[Tuple[int, int], List[float]], Dict[int, int], Dict[Tuple[int, int], int], Dict[Tuple[int, int], int]]:
        """Spin until enough valid frames are collected or timeout elapses."""
        per_motor: Dict[Tuple[int, int], List[float]] = {}
        per_chan_count: Dict[int, int] = {ch: 0 for ch in channels}
        skipped_err: Dict[Tuple[int, int], int] = {}
        skipped_sentinel: Dict[Tuple[int, int], int] = {}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            for ch in channels:
                msg = self._latest[ch]
                if msg is None:
                    continue
                key = id(msg)
                if self._last_id[ch] == key:
                    continue
                self._last_id[ch] = key
                per_chan_count[ch] += 1
                for m in msg.motors:
                    if m.channel != ch:
                        continue
                    slot = m.motor_index
                    motor_key = (ch, slot)
                    if motor_key not in limits:
                        continue
                    if not m.enabled:
                        skipped_err[motor_key] = skipped_err.get(motor_key, 0) + 1
                        continue
                    if not m.online or not all(math.isfinite(v) for v in (
                            m.position, m.velocity, m.torque)):
                        skipped_sentinel[motor_key] = skipped_sentinel.get(motor_key, 0) + 1
                        continue
                    per_motor.setdefault(motor_key, []).append(float(m.position))
        return per_motor, per_chan_count, skipped_err, skipped_sentinel


def main():
    argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parse_args(argv)

    if not args.offsets.exists():
        print(f'ERROR: YAML not found: {args.offsets}', file=sys.stderr)
        sys.exit(1)

    with args.offsets.open('r') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or 'offsets' not in data:
        print(f"ERROR: YAML must have a top-level 'offsets' list", file=sys.stderr)
        sys.exit(1)

    default_ch = int(data.get('channel', 0))
    channels = sorted({int(e.get('channel', default_ch)) for e in data['offsets']})
    if any(ch not in (0, 1) for ch in channels):
        raise ValueError('W3 channels must be 0/1; update the offsets file channel mapping')
    motor_type_map = {}
    if args.motor_type_map and args.motor_type_map.exists():
        with args.motor_type_map.open('r') as f:
            motor_type_map = yaml.safe_load(f) or {}

    limits: Dict[Tuple[int, int], Tuple[float, float, float]] = {}
    for e in data['offsets']:
        ch = int(e.get('channel', default_ch))
        slot = int(e['slot'])
        motor_type = e.get('motor_type') or motor_type_map.get(f'{ch}.{slot}') or motor_type_map.get(f'{ch}:{slot}')
        if motor_type not in MOTOR_LIMITS:
            print(f"ERROR: missing/unknown motor_type for ch{ch} slot{slot}; add motor_type in YAML or --motor-type-map", file=sys.stderr)
            sys.exit(1)
        limits[(ch, slot)] = MOTOR_LIMITS[motor_type]

    rclpy.init()
    try:
        node = RawCollector(channels)
        node.get_logger().info(
            f'Gathering {args.samples} valid samples per motor, timeout {args.timeout:.1f}s. '
            'Hold the robot at URDF zero pose, perfectly still...')
        per_motor, per_chan, skipped_err, skipped_sentinel = node.gather(channels, args.samples, args.timeout, limits)
        for ch in channels:
            if per_chan[ch] < args.samples:
                node.get_logger().warning(
                    f'ch{ch}: only got {per_chan[ch]} fresh frames (wanted {args.samples}). '
                    'Is the bridge running and are motors enabled?')
        node.destroy_node()
    finally:
        rclpy.shutdown()

    # Build summary table.
    print()
    print(f'{"name":<14} | {"ch":>2} | {"slot":>4} | '
          f'{"valid":>5} | {"raw_now":>10} | {"spread":>7} | '
          f'{"old_off":>10} | {"new_off":>10} | {"delta":>10} | '
          f'{"err":>5} | {"sent":>5}')
    print('-' * 125)

    n_changed = 0
    n_skipped = 0
    n_nodata = 0
    for e in data['offsets']:
        name = str(e.get('name', '?'))
        slot = int(e['slot'])
        ch = int(e.get('channel', default_ch))
        old = float(e.get('zero_offset', 0.0))
        sign = float(e.get('axis_sign', 1.0))
        raws = per_motor.get((ch, slot), [])
        err_skips = skipped_err.get((ch, slot), 0)
        sent_skips = skipped_sentinel.get((ch, slot), 0)
        if not raws:
            print(f'{name:<14} | {ch:>2} | {slot:>4} | '
                  f'{0:>5} | {"NO DATA":>10} | {"--":>7} | '
                  f'{old:+10.4f} | {"--":>10} | {"--":>10} | '
                  f'{err_skips:>5} | {sent_skips:>5}')
            n_nodata += 1
            continue
        spread = max(raws) - min(raws)
        avg = sum(raws) / len(raws)
        if spread > args.max_spread:
            print(f'{name:<14} | {ch:>2} | {slot:>4} | '
                  f'{len(raws):>5} | {avg:+10.4f} | {spread:>7.4f} | '
                  f'{old:+10.4f} | {"SKIP":>10} | {"--":>10} | '
                  f'{err_skips:>5} | {sent_skips:>5}  (jitter > {args.max_spread})')
            n_skipped += 1
            continue
        # zero_offset = raw_at_zero (regardless of axis_sign).
        new = avg
        delta = new - old
        e['zero_offset'] = float(new)
        # Annotate provenance, keep history fields if present.
        e['raw_at_reference'] = float(avg)
        e['urdf_pos_at_reference'] = 0.0
        e['reference'] = 'drag-to-zero'
        e['sample_count'] = int(len(raws))
        e['sample_spread'] = float(spread)
        print(f'{name:<14} | {ch:>2} | {slot:>4} | '
              f'{len(raws):>5} | {avg:+10.4f} | {spread:>7.4f} | '
              f'{old:+10.4f} | {new:+10.4f} | {delta:+10.4f} | '
              f'{err_skips:>5} | {sent_skips:>5}')
        n_changed += 1

    # Update top-level metadata.
    if n_changed:
        data['mode'] = 'drag-to-zero'
        data['note'] = (
            'zero_offset = raw at URDF zero pose. urdf_pos = axis_sign * '
            '(raw - zero_offset) -> 0 at this pose by construction. '
            'axis_sign must still be set manually if a motor is mounted reversed.')

    print()
    print(f'  changed: {n_changed}    skipped (jitter): {n_skipped}    no-data: {n_nodata}')
    if n_nodata > 0 or n_skipped > 0:
        print('WARNING: some joints lacked valid samples or were rejected; inspect the table above.')
    if args.apply:
        if n_changed == 0:
            print('Nothing to write.')
            return
        with args.offsets.open('w') as f:
            yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)
        print(f'  wrote {n_changed} updated zero_offset(s) to {args.offsets}')
    else:
        print(f'  DRY RUN. Re-run with --apply to write {args.offsets}.')


if __name__ == '__main__':
    main()
