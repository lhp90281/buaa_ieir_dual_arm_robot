#!/usr/bin/env python3
"""Mock W3 feedback for the real gripper executable; never open CAN."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = '217'

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import qos_profile_sensor_data
from w3_robot_bridge.msg import MotorCommand, MotorCommandArray, MotorState, MotorStateArray


def main():
    rclpy.init()
    node = rclpy.create_node('mock_gripper_transport')
    namespace = '/migration_test_' + uuid.uuid4().hex
    publisher = node.create_publisher(MotorStateArray, namespace + '/state',
                                      qos_profile_sensor_data)
    enabled = False
    stopping = False
    commands = []

    def receive(message):
        nonlocal enabled
        for command in message.commands:
            assert (command.channel, command.motor_index) == (0, 7)
            commands.append(command)
            if command.mode == MotorCommand.MODE_ENABLE:
                enabled = True
            elif command.mode == MotorCommand.MODE_DISABLE:
                enabled = False

    subscription = node.create_subscription(
        MotorCommandArray, namespace + '/commands', receive, qos_profile_sensor_data)

    def tick():
        msg = MotorStateArray()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.motors = [MotorState(channel=0, motor_index=i, online=True,
                                enabled=not stopping, error_flags=0 if stopping else 1)
                      for i in range(7)]
        msg.motors.append(MotorState(
            channel=0, motor_index=7, online=enabled or stopping,
            enabled=enabled, error_flags=1 if enabled else 0, position=1.2))
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)

    executable = Path(get_package_prefix('eiriarm_controllers')) / 'lib/eiriarm_controllers/gripper_controller_node'
    with tempfile.TemporaryFile(mode='w+') as output:
        process = subprocess.Popen([
            str(executable), '--ros-args', '-p', 'right_enabled:=false',
            '-r', '__node:=test_gripper',
            '-r', '/w3_robot_bridge_node/state:=' + namespace + '/state',
            '-r', '/w3_robot_bridge_node/commands:=' + namespace + '/commands',
        ], stdout=output, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline and process.poll() is None:
                tick()
                if any(command.kp > 0 for command in commands):
                    break
            assert any(command.mode == MotorCommand.MODE_ENABLE for command in commands), (
                'Gripper never enabled from initially offline feedback')
            assert any(command.kp > 0 for command in commands), 'Gripper never reached calibration'
            first_enable = next(i for i, c in enumerate(commands) if c.mode == MotorCommand.MODE_ENABLE)
            assert all(c.kp == 0 and c.torque == 0 for c in commands[:first_enable])
            print('PASS: offline -> enable -> valid feedback -> calibration, only can0 slot7')
        finally:
            stopping = True
            process.send_signal(signal.SIGINT)
            deadline = time.monotonic() + 15
            while process.poll() is None and time.monotonic() < deadline:
                tick()
            if process.poll() is None:
                process.kill()
            process.wait()
            output.seek(0)
            print(output.read())
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
