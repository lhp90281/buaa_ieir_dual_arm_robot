#!/usr/bin/env python3
"""Translate raw DM motor states to URDF joint states.

Pipeline:
    /w3_robot_bridge_node/state  (w3_robot_bridge/MotorStateArray, raw motor frame)
        |
        |  per joint (channel, slot):
        |      urdf = wrap_to_window(sign * (raw - zero_offset),
        |                            center = (urdf_lower + urdf_upper) / 2)
        |
        v
    /joint_states  (sensor_msgs/JointState, URDF frame)

Inputs:
    --offsets PATH    YAML produced by joint_zero_calibration (zero_offset per joint)
    --urdf    PATH    URDF/Xacro-evaluated XML file (joint <limit lower/upper>)

Per-joint optional field in offsets YAML (manually editable):
    axis_sign: +1 or -1   default +1; flip if motor's positive direction
                          doesn't match the URDF axis direction.

Wrap correction details:
    DM motors report a CONTINUOUS angle; the multi-turn accumulator resets on
    every power-cycle. So `raw` at the same physical position can differ by
    integer multiples of 2 pi between sessions. As long as each joint's URDF
    range is <= 2 pi (i.e. <= 1 revolution), folding raw-offset into the 2 pi
    window centred at the URDF range centre uniquely recovers urdf_pos.

    If any joint has range > 2 pi, this translator will warn at startup and
    skip wrap correction for that joint (output may then be incorrect after
    a wrap event; you should fix the URDF or use 0xFE save-zero instead).
"""
import argparse
import dataclasses
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from w3_robot_bridge.msg import MotorStateArray


@dataclasses.dataclass
class JointMap:
    name: str
    channel: int
    slot: int
    zero_offset: float
    axis_sign: float       # +1 or -1
    urdf_lower: float
    urdf_upper: float
    range_center: float
    wrap_safe: bool        # True if (upper - lower) <= 2 pi


def parse_urdf_joint_limits(urdf_path: Path) -> Dict[str, Dict[str, float]]:
    """Return {joint_name: {'lower': float, 'upper': float}} from a URDF file."""
    tree = ET.parse(str(urdf_path))
    root = tree.getroot()
    out: Dict[str, Dict[str, float]] = {}
    for joint in root.iter('joint'):
        name = joint.get('name')
        if not name:
            continue
        limit = joint.find('limit')
        if limit is None:
            continue
        try:
            lower = float(limit.get('lower'))
            upper = float(limit.get('upper'))
        except (TypeError, ValueError):
            continue
        out[name] = {'lower': lower, 'upper': upper}
    return out


def load_offsets_yaml(path: Path) -> Dict:
    if not path.exists():
        raise FileNotFoundError(f'Offsets YAML not found: {path}')
    with path.open('r') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or 'offsets' not in data:
        raise ValueError(f"{path} must contain a top-level 'offsets' list")
    return data


def raw_to_urdf(raw: float, offset: float, sign: float,
                range_center: float, wrap_safe: bool) -> float:
    """Map a raw motor reading to URDF joint position.

    If wrap_safe (URDF range <= 2 pi), folds into the 2 pi window centred at
    `range_center` so multi-turn ambiguity is resolved.
    Otherwise returns the unfolded value (may be off by 2 pi after a wrap).
    """
    urdf = sign * (raw - offset)
    if not wrap_safe:
        return urdf
    diff = (urdf - range_center + math.pi) % (2.0 * math.pi) - math.pi
    return range_center + diff


class JointStateTranslator(Node):
    def __init__(self, joints: List[JointMap], rate_hz: float, topic: str):
        super().__init__('joint_state_translator')
        self._joints = joints
        # latest MotorStateArray per channel
        self._latest: Dict[int, Optional[MotorStateArray]] = {}
        self._channels = sorted({j.channel for j in joints})
        for ch in self._channels:
            self._latest[ch] = None

        self._pub = self.create_publisher(JointState, topic, 10)

        self._sub = self.create_subscription(
            MotorStateArray, '/w3_robot_bridge_node/state',
            self._on_state, qos_profile_sensor_data)

        period = 1.0 / max(1.0, rate_hz)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'Translator ready: rate={rate_hz:.0f} Hz, output={topic}, '
            f'channels={self._channels}, joints={len(joints)}'
        )
        for j in joints:
            self.get_logger().info(
                f'  {j.name:<10s} ch{j.channel}.slot{j.slot}  '
                f'offset={j.zero_offset:+.4f}  sign={j.axis_sign:+.0f}  '
                f'urdf=[{j.urdf_lower:+.3f},{j.urdf_upper:+.3f}]  '
                f'center={j.range_center:+.3f}  wrap_safe={j.wrap_safe}'
            )

    def _on_state(self, msg: MotorStateArray):
        for ch in self._channels:
            self._latest[ch] = msg

    def _on_timer(self):
        # require at least one frame on every channel before publishing
        if any(self._latest[c] is None for c in self._channels):
            return
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        for j in self._joints:
            arr = self._latest[j.channel]
            if arr is None:
                continue
            m = next((m for m in arr.motors
                      if m.channel == j.channel and m.motor_index == j.slot
                      and m.online), None)
            if m is None or not all(math.isfinite(v) for v in (m.position, m.velocity, m.torque)):
                continue
            urdf_pos = raw_to_urdf(
                m.position, j.zero_offset, j.axis_sign,
                j.range_center, j.wrap_safe,
            )
            urdf_vel = j.axis_sign * m.velocity
            urdf_eff = j.axis_sign * m.torque
            out.name.append(j.name)
            out.position.append(urdf_pos)
            out.velocity.append(urdf_vel)
            out.effort.append(urdf_eff)
        self._pub.publish(out)


def build_joint_maps(offsets_data: Dict, urdf_limits: Dict[str, Dict[str, float]],
                     logger=None) -> List[JointMap]:
    default_channel = int(offsets_data.get('channel', 0))
    out: List[JointMap] = []
    for entry in offsets_data['offsets']:
        name = entry['name']
        slot = int(entry['slot'])
        ch = int(entry.get('channel', default_channel))
        if ch not in (0, 1):
            raise ValueError('W3 channel must be 0 (left) or 1 (right)')
        zero_offset = float(entry['zero_offset'])
        axis_sign = float(entry.get('axis_sign', 1.0))
        if axis_sign not in (1.0, -1.0):
            raise ValueError(f"{name}: axis_sign must be +1 or -1, got {axis_sign}")

        if name not in urdf_limits:
            msg = (f"{name}: not found in URDF; using fallback limits "
                   "[-pi, +pi]. Wrap correction will assume center=0.")
            if logger:
                logger.warning(msg)
            else:
                print('WARNING:', msg, file=sys.stderr)
            lower, upper = -math.pi, math.pi
        else:
            lower = urdf_limits[name]['lower']
            upper = urdf_limits[name]['upper']

        rng = upper - lower
        wrap_safe = rng <= 2.0 * math.pi + 1e-9
        if not wrap_safe and logger:
            logger.warning(
                f"{name}: URDF range = {rng:.3f} rad > 2pi; "
                "wrap correction disabled for this joint."
            )
        out.append(JointMap(
            name=name, channel=ch, slot=slot,
            zero_offset=zero_offset, axis_sign=axis_sign,
            urdf_lower=lower, urdf_upper=upper,
            range_center=0.5 * (lower + upper),
            wrap_safe=wrap_safe,
        ))
    return out


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Translate raw DM motor states to URDF joint states.'
    )
    p.add_argument('--offsets', type=Path, default=Path('joint_offsets.yaml'),
                   help='Path to joint_offsets.yaml from joint_zero_calibration. '
                        'Default: joint_offsets.yaml in CWD.')
    p.add_argument('--urdf', type=Path, required=True,
                   help='Path to URDF XML file (limits parsed for wrap window).')
    p.add_argument('--rate', type=float, default=200.0,
                   help='Output publish rate in Hz (default 200).')
    p.add_argument('--topic', type=str, default='/joint_states',
                   help='Output JointState topic (default /joint_states).')
    return p.parse_args(argv)


def main():
    # rclpy strips its own --ros-args; keep our args separate.
    argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parse_args(argv)

    offsets_data = load_offsets_yaml(args.offsets)
    urdf_limits = parse_urdf_joint_limits(args.urdf)

    rclpy.init()
    try:
        joints = build_joint_maps(offsets_data, urdf_limits, logger=None)
        node = JointStateTranslator(joints, args.rate, args.topic)
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
