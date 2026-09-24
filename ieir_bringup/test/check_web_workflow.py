#!/usr/bin/env python3
"""Managed headless calibration, isolated fake motors. Never launches bridge/control."""
import os
os.environ['ROS_DOMAIN_ID'] = '218'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

from pathlib import Path
import shutil
import sys
import tempfile
import time

import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from w3_robot_bridge.msg import MotorCommand, MotorCommandArray, MotorState, MotorStateArray

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ieir_bringup/scripts'))
from web_console import ConsoleApp


def main():
    with tempfile.TemporaryDirectory(prefix='web_calibration_test_') as temporary:
        workspace = Path(temporary)
        directory = workspace / 'src/ros2_ws_config'
        directory.mkdir(parents=True)
        (workspace / 'src/description').symlink_to(ROOT / 'description', target_is_directory=True)
        for name in ('joint_calibration_dual_right.yaml', 'merge_offsets.py'):
            shutil.copy2(ROOT / 'ros2_ws_config' / name, directory / name)
        app = ConsoleApp(workspace, ROOT / 'ieir_bringup/web', allow_control=True, manage_processes=True)
        bus = Node('web_workflow_fake_bridge')
        app.backend.executor.add_node(bus)
        enabled = [False] * 7
        positions = [2.0] * 7
        transitions, failures = [], []
        def command(message):
            for m in message.commands:
                if m.channel != 1 or not 0 <= m.motor_index < 7 or any((m.kp, m.kd, m.torque, m.velocity)):
                    failures.append('Non-passive or wrong-channel calibration command')
                    continue
                i = m.motor_index
                if m.mode == MotorCommand.MODE_ENABLE:
                    if not enabled[i]:
                        transitions.append(('enable', i))
                    enabled[i] = True
                elif m.mode == MotorCommand.MODE_DISABLE:
                    if enabled[i]:
                        transitions.append(('disable', i))
                    enabled[i] = False
        bus.create_subscription(MotorCommandArray, '/w3_robot_bridge_node/commands', command, 10)
        publisher = bus.create_publisher(MotorStateArray, '/w3_robot_bridge_node/state', qos_profile_sensor_data)
        def feedback():
            message = MotorStateArray()
            message.header.stamp = bus.get_clock().now().to_msg()
            message.motors = [MotorState(channel=1, motor_index=i, online=True, enabled=enabled[i],
                error_flags=1 if enabled[i] else 0, position=positions[i]) for i in range(7)]
            publisher.publish(message)
        bus.create_timer(.02, feedback)
        def action(action, **kwargs):
            return app.action(dict(action=action, confirmed=True, **kwargs))
        def wait(predicate, timeout=15, heartbeat=True):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if heartbeat:
                    action('heartbeat')
                if predicate():
                    assert not failures, failures
                    return
                time.sleep(.04)
            raise AssertionError('Workflow timeout; logs:\n' + '\n'.join(e['text'] for e in app.state.snapshot()['logs']))
        def session(stage, joint=None, phase=None):
            items = app.snapshot()['calibration']
            for prefix, value in items.items():
                m = value.get('metadata', {})
                if (value['age'] < 1 and m.get('stage') == stage and
                        (joint is None or m.get('joint') == f'right_joint_{joint}') and
                        (phase is None or m.get('phase') == phase)):
                    return prefix, value
            return None
        def start(stage, **kwargs):
            wait(lambda: not app.state.calibration and app.backend.count_publishers('/w3_robot_bridge_node/commands') == 0)
            action('process_start', kind='calibration', side='right', stage=stage, **kwargs)
        def finished():
            wait(lambda: not app.processes.active('calibration'))
            assert app.processes.snapshot()['calibration']['exit_code'] == 0
            wait(lambda: not any(enabled))
        try:
            wait(lambda: len(app.state.motors) == 7)
            assert not transitions, 'Opening UI must not enable anything'
            start('direction')
            for i in range(7):
                wait(lambda: session('direction', i))
                prefix, _ = session('direction', i)
                action('calibration_key', session=prefix, key='enter')
                wait(lambda: 'Move this joint gently' in (session('direction', i) or ('', {}))[1].get('text', ''))
                positions[i] += .12
                wait(lambda: abs((session('direction', i) or ('', {}))[1].get('metadata', {}).get('angle_deg', 0)) > 5)
                if i == 0:
                    action('calibration_key', session=prefix, key='space')
                    wait(lambda: 'sign=-1' in session('direction', i)[1]['text'])
                action('calibration_key', session=prefix, key='enter')
            finished()
            directions = yaml.safe_load((directory / 'joint_directions_right.yaml').read_text())
            assert [e['axis_sign'] for e in directions['directions']] == [-1] + [1] * 6
            print('PASS direction: headless, sign flip, seven confirmations, disable/save', flush=True)

            before = list(transitions)
            start('limits')
            for i in (6, 5, 4, 2, 3, 1, 0):
                wait(lambda: session('limits', i, 'review'))
                prefix, _ = session('limits', i)
                if i == 6:
                    action('calibration_angle', session=prefix, angle_deg=-89.5)
                    wait(lambda: session('limits', i)[1]['metadata']['angle_deg'] == -89.5)
                action('calibration_key', session=prefix, key='enter')
            finished()
            assert transitions == before, 'Reference editing must not enable motors'
            print('PASS limits: browser angle editing, seven confirmations, no motor commands', flush=True)

            positions[:] = [2.0] * 7
            before = len(transitions)
            start('manual')
            for i in (6, 5, 4, 2, 3, 1, 0):
                wait(lambda: session('manual', i, 'arm'))
                prefix, _ = session('manual', i)
                action('calibration_key', session=prefix, key='enter')
                wait(lambda: session('manual', i, 'manual'))
                assert sum(enabled) == 1 and enabled[i]
                time.sleep(1)
                action('calibration_key', session=prefix, key='enter')
                wait(lambda: session('manual', i, 'manual_preview'))
                saved = yaml.safe_load((directory / 'joint_offsets_right.yaml').read_text())
                assert any(e['slot'] == i and e['method'] == 'manual' for e in saved['offsets'])
                positions[i] += .05
                wait(lambda: abs(session('manual', i)[1].get('positions', {}).get(f'right_joint_{i}', 0)
                                 - session('manual', i)[1]['metadata']['angle_deg'] * 3.141592653589793 / 180) > .04)
                action('calibration_key', session=prefix, key='n')
            finished()
            expected = [(op, i) for i in (6, 5, 4, 2, 3, 1, 0) for op in ('enable', 'disable')]
            assert transitions[before:] == expected
            assert saved['complete'] and len(saved['offsets']) == 7
            print('PASS manual: 7-6-5-3-4-2-1, single enabled motor, save then delta preview, no PD', flush=True)

            start('manual', joint=7)
            wait(lambda: session('manual', 6, 'arm'))
            prefix, _ = session('manual', 6)
            action('calibration_key', session=prefix, key='enter')
            wait(lambda: enabled[6] and session('manual', 6, 'manual'))
            wait(lambda: not app.processes.active('calibration'), heartbeat=False)
            wait(lambda: not any(enabled))
            assert list((workspace / 'log/web_calibration_backups').glob('*joint_offsets_right.yaml'))
            print('PASS browser disappearance: abort/disable; existing results backed up and retained', flush=True)

            start('manual', joint=7)
            wait(lambda: session('manual', 6, 'arm'))
            prefix, _ = session('manual', 6)
            action('calibration_key', session=prefix, key='enter')
            wait(lambda: enabled[6] and session('manual', 6, 'manual'))
            action('process_stop', kind='calibration')
            wait(lambda: not any(enabled) and not app.state.calibration)
            assert not app.processes.active('calibration')
            print('PASS owned process stop: SIGINT completes motor disable cleanup', flush=True)

            # Synthetic second arm, never copy actual per-robot calibration.
            left = yaml.safe_load((directory / 'joint_offsets_right.yaml').read_text())
            left['channel'] = 0
            for entry in left['offsets']:
                entry['name'] = entry['name'].replace('right_', 'left_')
                entry['channel'] = 0
            (directory / 'joint_offsets_left.yaml').write_text(yaml.safe_dump(left))
            action('process_start', kind='merge')
            wait(lambda: not app.processes.active('merge'))
            assert app.processes.snapshot()['merge']['exit_code'] == 0
            merged = yaml.safe_load((directory / 'joint_offsets_dual.yaml').read_text())
            assert len(merged['offsets']) == 14 and merged['transport'] == 'w3'
            print('PASS explicit merge: both arms, 14 named offsets, no motor commands', flush=True)
        finally:
            app.close()
            bus.destroy_node()


if __name__ == '__main__':
    main()
