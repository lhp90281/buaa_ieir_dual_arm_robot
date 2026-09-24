#!/usr/bin/env python3
"""GUI + real calibration script smoke test using fake motors in an isolated ROS domain."""
import os
os.environ['ROS_DOMAIN_ID'] = '219'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import re
import signal
import sys
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from w3_robot_bridge.msg import MotorCommand, MotorCommandArray, MotorState, MotorStateArray
import yaml

ROOT = Path(__file__).resolve().parents[3]


def main():
    abort = '--abort' in sys.argv
    rclpy.init()
    node = rclpy.create_node('fake_calibration_motors')
    states = [0] * 7
    positions = [5.0] * 7
    enabled_seen = False
    disabled_seen = False

    def command(msg):
        nonlocal enabled_seen, disabled_seen
        for m in msg.commands:
            assert m.channel == 1 and 0 <= m.motor_index < 7
            if m.mode == MotorCommand.MODE_ENABLE:
                states[m.motor_index] = 1
                enabled_seen = True
            elif m.mode == MotorCommand.MODE_DISABLE:
                states[m.motor_index] = 0
                disabled_seen = True
            else:
                assert m.mode == MotorCommand.MODE_RUN
                assert m.kp == m.kd == m.torque == 0.0

    commands = node.create_subscription(MotorCommandArray, '/w3_robot_bridge_node/commands', command, 10)
    feedback = node.create_publisher(MotorStateArray, '/w3_robot_bridge_node/state', qos_profile_sensor_data)
    keys = None
    status_sub = None
    stage = {}
    moved = set()

    def status(msg):
        match = re.search(r'right_joint_(\d)', msg.data)
        if not match or keys is None:
            return
        slot = int(match.group(1))
        if stage.get(slot, 0) == 0 and 'ENTER captures' in msg.data:
            keys.publish(String(data='enter'))
            stage[slot] = 1
        elif stage.get(slot) == 1 and 'Move this joint gently' in msg.data:
            moved.add(slot)
            delta = re.search(r'delta=([+-][\d.]+)', msg.data)
            if delta and abs(float(delta.group(1))) >= 6:
                keys.publish(String(data='escape' if abort else ('space' if slot == 0 else 'enter')))
                stage[slot] = 2
        elif stage.get(slot) == 2 and slot == 0 and 'sign=-1' in msg.data:
            keys.publish(String(data='enter'))
            stage[slot] = 3

    with tempfile.TemporaryDirectory(prefix='direction_test_') as directory:
        output = Path(directory) / 'joint_directions_right.yaml'
        log = Path(directory) / 'calibration.log'
        with log.open('w') as stream:
            process = subprocess.Popen([
                'ros2', 'run', 'ieir_controllers', 'joint_zero_calibration',
                '--mode', 'direction', '--calibration-yaml',
                str(ROOT / 'ros2_ws_config/joint_calibration_dual_right.yaml'),
                '--preview-backend', 'mujoco', '--output', str(output)], stdout=stream, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 45
                while process.poll() is None and time.monotonic() < deadline:
                    if keys is None:
                        for topic, _ in node.get_topic_names_and_types():
                            if topic.startswith('/calibration_preview_') and topic.endswith('/status'):
                                prefix = topic[:-len('/status')]
                                keys = node.create_publisher(String, prefix + '/key', 10)
                                status_sub = node.create_subscription(String, topic, status, 10)
                                break
                    msg = MotorStateArray()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    for slot in range(7):
                        if slot in moved:
                            positions[slot] = min(positions[slot] + 0.006, 5.14)
                        msg.motors.append(MotorState(channel=1, motor_index=slot, online=True,
                            enabled=states[slot] == 1, error_flags=states[slot], position=positions[slot]))
                    feedback.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.01)
                    time.sleep(0.005)
                if process.poll() is None:
                    raise RuntimeError('Direction workflow timeout')
                if abort:
                    assert process.returncode != 0, log.read_text()
                    assert not output.exists(), 'Aborted calibration must not save partial signs'
                else:
                    assert process.returncode == 0, log.read_text()
                    data = yaml.safe_load(output.read_text())
                    assert data['mode'] == 'direction' and data['channel'] == 1
                    assert [e['axis_sign'] for e in data['directions']] == [-1] + [1] * 6
                    assert all(e['direction_verified'] for e in data['directions'])
                assert enabled_seen and disabled_seen and states == [0] * 7
                print('PASS: abort -> disable -> no partial output' if abort else
                      'PASS: GUI ready -> enable -> seven delta checks -> sign flip -> disable -> direction file')
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
