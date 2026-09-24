#!/usr/bin/env python3
"""Headless plugin smoke test with synthetic state; never starts W3 or CAN."""
import os
os.environ['ROS_DOMAIN_ID'] = '221'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import math
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = rclpy.create_node('ieir_simulation_test')
    publisher = node.create_publisher(JointState, '/joint_states', 10)
    messages = {}
    subscriptions = [node.create_subscription(
        JointState, topic, lambda msg, topic=topic: messages.update({topic: msg}), 10)
        for topic in ('/ctrl/command', '/ctrl/gains')]
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    names = [f'{side}_joint_{i}' for side in ('left', 'right') for i in range(7)]
    with tempfile.TemporaryDirectory(prefix='ieir_simulation_') as directory:
        log = Path(directory) / 'controllers.log'
        with log.open('w') as stream:
            process = subprocess.Popen(
                ['ros2', 'launch', 'ieir_simulation', 'controllers.launch.py'],
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline, future, active = time.monotonic() + 30, None, set()
                while time.monotonic() < deadline:
                    assert process.poll() is None, log.read_text()
                    publisher.publish(JointState(
                        name=names, position=[0.0] * 14, velocity=[0.0] * 14, effort=[0.0] * 14))
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if future is None and client.service_is_ready():
                        future = client.call_async(ListControllers.Request())
                    if future is not None and future.done():
                        active = {c.name for c in future.result().controller if c.state == 'active'}
                        future = None
                    if {'gravity_compensation_controller', 'joint_position_controller'} <= active and len(messages) == 2:
                        break
                assert {'gravity_compensation_controller', 'joint_position_controller'} <= active, log.read_text()
                assert len(messages) == 2, log.read_text()
                for topic, msg in messages.items():
                    assert set(msg.name) == set(names), (topic, msg)
                    assert all(math.isfinite(v) for values in (msg.position, msg.velocity, msg.effort) for v in values)
                assert not any('w3_robot_bridge' in n or 'gripper_controller' in n for n in node.get_node_names())
                print('PASS: simulation topic hardware + shared gravity/joint plugins active; finite MIT commands, no CAN nodes')
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=15)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
