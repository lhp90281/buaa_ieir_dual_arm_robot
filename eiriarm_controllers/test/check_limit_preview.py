#!/usr/bin/env python3
"""Exercise the real read-only GUI in an isolated ROS domain; never connects to CAN."""
import os
os.environ['ROS_DOMAIN_ID'] = '218'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import math
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from std_msgs.msg import String
import yaml

ROOT = Path(__file__).resolve().parents[2]


def main():
    abort = '--abort' in sys.argv
    rclpy.init()
    node = rclpy.create_node('limit_preview_test')
    keys = None
    status_sub = None
    seen = []
    edited = False

    def status(msg):
        nonlocal edited
        match = re.search(r'Joint (\d) \(right_joint_', msg.data)
        if not match or keys is None or not keys.get_subscription_count():
            return
        joint = int(match.group(1))
        if joint in seen:
            return
        if not edited:
            keys.publish(String(data='angle_deg:-85'))
            edited = True
            return
        if joint == 7 and 'Reference: -85.00 deg' not in msg.data:
            return
        seen.append(joint)
        keys.publish(String(data='escape' if abort else 'enter'))

    source = ROOT / 'ros2_ws_config/joint_calibration_dual_right.yaml'
    original = source.read_bytes()
    with tempfile.TemporaryDirectory(prefix='limit_preview_test_') as directory:
        output = Path(directory) / 'reviewed.yaml'
        if abort:
            output.write_text('existing: unchanged\n')
        log = Path(directory) / 'preview.log'
        with log.open('w') as stream:
            process = subprocess.Popen([
                'ros2', 'run', 'eiriarm_controllers', 'joint_zero_calibration',
                '--mode', 'limit-preview', '--calibration-yaml', str(source),
                '--output', str(output)], stdout=stream, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 40
                while process.poll() is None and time.monotonic() < deadline:
                    assert node.count_publishers('/w3_robot_bridge_node/commands') == 0
                    if keys is None:
                        for topic, _ in node.get_topic_names_and_types():
                            if topic.startswith('/calibration_preview_') and topic.endswith('/status'):
                                keys = node.create_publisher(String, topic[:-7] + '/key', 10)
                                status_sub = node.create_subscription(String, topic, status, 10)
                                break
                    rclpy.spin_once(node, timeout_sec=0.02)
                assert process.poll() is not None, 'Preview timed out'
                assert source.read_bytes() == original
                if abort:
                    assert process.returncode != 0, log.read_text()
                    assert output.read_text() == 'existing: unchanged\n'
                else:
                    assert process.returncode == 0, log.read_text()
                    assert seen == [7, 6, 5, 3, 4, 2, 1]
                    data = yaml.safe_load(output.read_text())
                    joints = {j['slot']: j for j in data['joints']}
                    assert abs(joints[6]['urdf_pos_at_limit'] - math.radians(-85)) < 1e-10
                    assert all(j['reference_review']['confirmed'] for j in joints.values())
                print('PASS: edit + cancel preserves file; no command publisher' if abort else
                      'PASS: edit + confirm seven references + save; no command publisher')
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
