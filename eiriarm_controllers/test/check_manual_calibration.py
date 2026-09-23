#!/usr/bin/env python3
"""Exercise manual seven-joint calibration with fake feedback; never connects to CAN."""
import os
os.environ['ROS_DOMAIN_ID'] = '217'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from w3_robot_bridge.msg import MotorCommand, MotorCommandArray, MotorState, MotorStateArray
import yaml
from calibration_test_data import write_test_inputs


ROOT = Path(__file__).resolve().parents[2]


def main():
    exit_after_capture = '--exit-after-capture' in sys.argv
    skip_existing = '--skip-existing' in sys.argv
    rclpy.init()
    node = rclpy.create_node('fake_manual_calibration_motors')
    refs, directions = None, None
    originals = []
    order = [6, 5, 4, 2, 3, 1, 0]
    enabled = {s: False for s in order}
    raw = {s: 2.0 for s in enabled}
    enable_order, disable_order = [], []
    topics, held = {}, []
    phase_times, pressed = {}, set()
    preview_seen = set()
    captured = set()
    output = None

    def command(msg):
        assert len(msg.commands) == 1
        m = msg.commands[0]
        assert m.channel == 1 and m.motor_index in enabled
        assert m.kp == m.kd == m.velocity == m.torque == 0.0, 'Manual mode must never drive'
        slot = m.motor_index
        if m.mode == MotorCommand.MODE_ENABLE:
            assert not any(on for s, on in enabled.items() if s != slot)
            if not enabled[slot]:
                enable_order.append(slot)
            enabled[slot] = True
        elif m.mode == MotorCommand.MODE_DISABLE:
            if enabled[slot]:
                disable_order.append(slot)
            enabled[slot] = False
        else:
            assert m.mode == MotorCommand.MODE_RUN

    def status(msg, prefix):
        match = re.search(r'Joint (\d+)  \|  ([A-Z_]+)', msg.data)
        if not match:
            return
        slot, phase = int(match.group(1)) - 1, match.group(2)
        assert slot in enabled
        assert phase in ('ARM', 'ENABLING', 'MANUAL', 'MANUAL_PREVIEW')
        item = topics[prefix]
        item.update(slot=slot, phase=phase)
        phase_times.setdefault((slot, phase), time.monotonic())
        if phase == 'MANUAL_PREVIEW':
            data = yaml.safe_load(output.read_text())
            captured.add(slot)
            assert all(e in data['offsets'] for e in originals if e['slot'] not in captured), 'Preserve other results'
            entry = next(e for e in data['offsets'] if e['slot'] == slot)
            expected_offset = 2 - directions[slot]['axis_sign'] * refs['joints'][slot]['urdf_pos_at_limit']
            assert abs(entry['zero_offset'] - expected_offset) < 1e-6
            assert entry['method'] == 'manual' and not entry['visually_verified']
            assert not entry['retreats']

    def pose(msg, prefix):
        item = topics[prefix]
        if 'slot' not in item:
            return
        slot, phase = item['slot'], item['phase']
        assert msg.name == [f'right_joint_{slot}']
        reference = refs['joints'][slot]['urdf_pos_at_limit']
        if phase == 'MANUAL':
            assert abs(msg.position[0] - reference) < 1e-6, 'Before capture show expected limit'
        elif phase == 'MANUAL_PREVIEW':
            moved = directions[slot]['axis_sign'] * (msg.position[0] - reference)
            if moved > 0.02:
                preview_seen.add(slot)

    held.append(node.create_subscription(MotorCommandArray, '/w3_robot_bridge_node/commands', command, 10))
    feedback = node.create_publisher(MotorStateArray, '/w3_robot_bridge_node/state', qos_profile_sensor_data)
    with tempfile.TemporaryDirectory(prefix='manual_calibration_test_') as directory:
        reference_path, direction_path = write_test_inputs(directory)
        refs = yaml.safe_load(reference_path.read_text())
        directions = yaml.safe_load(direction_path.read_text())['directions']
        originals = [dict(slot=s, name=f'right_joint_{s}', axis_sign=directions[s]['axis_sign'],
                          zero_offset=0.123, method='repeated_position_pd_seek') for s in (6, 5, 4, 2)]
        output = Path(directory) / 'offsets.yaml'
        output.write_text(yaml.safe_dump(dict(channel=1, offsets=originals)))
        log = Path(directory) / 'run.log'
        with log.open('w') as stream:
            process = subprocess.Popen([
                'ros2', 'run', 'eiriarm_controllers', 'joint_manual_calibration',
                '--calibration-yaml', str(reference_path),
                '--directions-yaml', str(direction_path),
                '--output', str(output)], stdout=stream, stderr=subprocess.STDOUT)
            try:
                deadline, last = time.monotonic() + 90, time.monotonic()
                while process.poll() is None and time.monotonic() < deadline:
                    now = time.monotonic()
                    dt, last = min(0.03, now - last), now
                    for topic, _ in node.get_topic_names_and_types():
                        if topic.startswith('/calibration_preview_') and topic.endswith('/status'):
                            prefix = topic[:-7]
                            if prefix not in topics:
                                topics[prefix] = dict(keys=node.create_publisher(String, prefix + '/key', 10))
                                held.append(node.create_subscription(String, topic,
                                    lambda msg, p=prefix: status(msg, p), 10))
                                held.append(node.create_subscription(JointState, prefix + '/pose',
                                    lambda msg, p=prefix: pose(msg, p), 10))
                    for prefix, item in topics.items():
                        if 'slot' not in item or not item['keys'].get_subscription_count():
                            continue
                        # Our observation subscriptions must not stand in for a
                        # fully initialized viewer when auto-pressing keys.
                        if (node.count_publishers(prefix + '/key') < 2
                                or node.count_subscribers(prefix + '/pose') < 2
                                or node.count_subscribers(prefix + '/status') < 2):
                            continue
                        slot, phase = item['slot'], item['phase']
                        elapsed = now - phase_times[(slot, phase)]
                        if phase == 'MANUAL_PREVIEW':
                            raw[slot] += 0.04 * dt
                        key = ('s' if phase == 'ARM' and skip_existing and slot in (6, 5, 4, 2) else
                               'enter' if phase == 'ARM' else
                               'enter' if phase == 'MANUAL' and elapsed > 1.3 else
                               ('escape' if exit_after_capture else 'n')
                               if phase == 'MANUAL_PREVIEW' and slot in preview_seen else None)
                        if key and (slot, phase) not in pressed:
                            item['keys'].publish(String(data=key))
                            pressed.add((slot, phase))
                    msg = MotorStateArray()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.motors = [MotorState(channel=1, motor_index=s, online=True, enabled=on,
                        error_flags=1 if on else 0, position=raw[s], velocity=0.0, torque=0.0)
                        for s, on in enabled.items()]
                    feedback.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.005)
                    time.sleep(0.005)
                assert process.poll() == 0, log.read_text()
                assert 'terminate called' not in log.read_text(), log.read_text()
                expected = [3, 1, 0] if skip_existing else order
                if exit_after_capture:
                    expected = expected[:1]
                assert enable_order == disable_order == expected
                assert not any(enabled.values()) and preview_seen == set(expected)
                saved = yaml.safe_load(output.read_text())
                slots = {e['slot'] for e in originals} | set(expected)
                assert saved['complete'] == (len(slots) == 7)
                assert len(saved['offsets']) == len(slots)
                assert all(e in saved['offsets'] for e in originals if e['slot'] not in captured)
                print('PASS: saved reference survives ESC; next joint not enabled' if exit_after_capture else
                      'PASS: manual group, zero impedance, fixed reference -> one capture/save -> live preview; untouched results preserved')
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=6)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                print(log.read_text())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
