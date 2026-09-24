"""Offline console tests. Fake backend only, never start ROS or CAN."""
import importlib.util
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ieir_bringup/scripts'))
from web_console_core import (CARTESIAN, GRAVITY, JOINT, ConsoleState, calibration_files,
                              number, robot_model, switch_plan, zero_trajectory, display_assets)
from web_console import ConsoleApp, handler_for


@pytest.fixture
def state():
    model, _ = robot_model(ROOT / 'description')
    state = ConsoleState(model)
    for name in state.limits:
        state.joints[name] = dict(position=0.0, velocity=0.0, effort=0.0, at=time.monotonic())
    state.controllers = {GRAVITY: 'active', JOINT: 'active', CARTESIAN: 'inactive'}
    state.controller_at = time.monotonic()
    return state


def test_model_references_existing_local_assets(state):
    model, assets = robot_model(ROOT / 'description')
    assert len(state.limits) == 14
    assert all(p.is_file() for p in assets.values())
    assert not any('gripper' in link['name'] for link in model['links'])
    assert all(v['mesh'].startswith('/mesh/') for link in model['links'] for v in link['visuals'])


def test_display_lod_matches_source_fingerprint_and_falls_back(tmp_path):
    directory = ROOT / 'ieir_bringup/web/meshes'
    _, assets = robot_model(ROOT / 'description')
    lightweight = display_assets(assets, directory)
    assert set(lightweight) == set(assets)
    assert all(p.is_file() and p.is_relative_to(directory) for p in lightweight.values())
    assert sum(p.stat().st_size for p in lightweight.values()) < sum(p.stat().st_size for p in assets.values()) * .5
    changed = tmp_path / 'custom.stl'
    changed.write_bytes(b'different user mesh')
    assert display_assets({'test': changed}, directory) == {'test': changed}
    (tmp_path / 'manifest.json').write_text(json.dumps({hashlib.sha256(changed.read_bytes()).hexdigest():
                                                      {'file': '../not-allowed.stl'}}))
    assert display_assets({'test': changed}, tmp_path) == {'test': changed}


@pytest.mark.parametrize('value', [None, True, '1', float('nan'), float('inf'), -2, 2])
def test_numbers_reject_bad_input(value):
    with pytest.raises(ValueError):
        number(value, -1, 1)


def test_gravity_is_never_deactivated_by_switch():
    controllers = {GRAVITY: 'active', JOINT: 'active', CARTESIAN: 'inactive'}
    assert switch_plan(controllers, 'cartesian') == ([CARTESIAN], [JOINT])
    assert switch_plan(controllers, 'joint') == ([], [])
    assert switch_plan(controllers, 'gravity') == ([], [JOINT])
    with pytest.raises(ValueError):
        switch_plan({}, 'joint')


def test_joint_targets_fail_closed(state):
    target = dict(positions={'left_joint_0': 0.1}, duration=3)
    assert state.joint_target(target) == (target['positions'], 3)
    for bad in [dict(positions={'left_joint_0': 1.0}, duration=3),
                dict(positions={'waist_joint': 0}, duration=3),
                dict(positions={'left_joint_0': 0.1}, duration=0.1),
                dict(positions={'left_joint_0': float('nan')}, duration=3)]:
        with pytest.raises(ValueError):
            state.joint_target(bad)
    state.joints['left_joint_0']['at'] -= 1
    with pytest.raises(ValueError, match='Stale'):
        state.joint_target(target)


def test_inactive_gravity_and_stale_manager_block_motion(state):
    target = dict(positions={'left_joint_0': 0.1}, duration=3)
    state.controllers[GRAVITY] = 'inactive'
    with pytest.raises(ValueError, match='active'):
        state.joint_target(target)
    state.controllers[GRAVITY] = 'active'
    state.controller_at -= 4
    with pytest.raises(ValueError, match='active'):
        state.joint_target(target)


def test_zero_plan_selects_only_requested_arm_and_caps_peak_speed(state):
    state.joints['right_joint_0']['position'] = 1.5
    current, duration = state.zero_plan(dict(side='right', duration=3))
    assert len(current) == 7 and all(n.startswith('right_') for n in current)
    assert duration > 8
    points = list(zero_trajectory(current, duration))
    assert points[0] == (0.0, list(current.values()))
    assert points[-1] == (duration, [0.0] * 7)
    assert all(0 < b[0] - a[0] <= 0.020001 for a, b in zip(points, points[1:]))
    assert max(abs(b[1][0]-a[1][0])/(b[0]-a[0]) for a, b in zip(points, points[1:])) <= math.radians(20)
    assert len(state.zero_plan(dict(side='dual', duration=8))[0]) == 14
    # The dedicated zero path does not relax the arbitrary-target limit.
    with pytest.raises(ValueError, match='20 degrees'):
        state.joint_target(dict(positions={'right_joint_0': 0.0}, duration=8))


@pytest.mark.parametrize('side,duration', [('waist', 8), ('left', 0), ('dual', float('nan'))])
def test_zero_plan_rejects_invalid_input(state, side, duration):
    with pytest.raises(ValueError):
        state.zero_plan(dict(side=side, duration=duration))


def test_zero_requires_all_selected_feedback_and_valid_model_zero(state):
    state.joints.pop('left_joint_6')
    with pytest.raises(ValueError, match='missing'):
        state.zero_plan(dict(side='dual', duration=8))
    assert len(state.zero_plan(dict(side='right', duration=8))[0]) == 7
    state.limits['right_joint_6'] = (0.1, 1)
    with pytest.raises(ValueError):
        state.zero_plan(dict(side='right', duration=8))


def test_pose_stream_packet_omits_stale_and_invalid_joints(state):
    state.joints['left_joint_0']['at'] -= 1
    state.joints['right_joint_0']['position'] = float('nan')
    packet = state.pose_sample()
    assert len(packet['positions']) == 12
    assert 'left_joint_0' not in packet['positions']
    assert 'right_joint_0' not in packet['positions']
    assert 'logs' not in packet


def test_cartesian_frame_freshness_and_delta_limits(state):
    state.controllers[CARTESIAN] = 'active'
    data = dict(side='right', position=[0.02, 0, 0], quaternion=[0, 0, 0, 1])
    with pytest.raises(ValueError, match='TF'):
        state.cartesian_target(data)
    state.poses['right'] = dict(at=time.monotonic(), position=[0, 0, 0], quaternion=[0, 0, 0, 1])
    assert state.cartesian_target(data)[0] == 'right'
    with pytest.raises(ValueError, match='50 mm'):
        state.cartesian_target({**data, 'position': [0.06, 0, 0]})
    with pytest.raises(ValueError, match='10 degrees'):
        state.cartesian_target({**data, 'quaternion': [0, 0, math.sin(.2), math.cos(.2)]})
    with pytest.raises(ValueError, match='normalized'):
        state.cartesian_target({**data, 'quaternion': [0, 0, 0, 0]})


def test_configuration_works_without_machine_calibration(tmp_path):
    files = calibration_files(tmp_path)
    assert len(files) == 8
    assert all(not f['exists'] for f in files)


@pytest.fixture
def app(tmp_path):
    (tmp_path / 'src').mkdir()
    (tmp_path / 'src/description').symlink_to(ROOT / 'description', target_is_directory=True)
    application = ConsoleApp(tmp_path, ROOT / 'ieir_bringup/web', preview=True)
    yield application
    application.close()


def test_console_serves_full_original_meshes(app):
    _, original = robot_model(app.workspace / 'src/description')
    assert app.assets == original
    assert all('web/meshes' not in str(path) for path in app.assets.values())


def test_read_only_refuses_ros_action(app):
    app.allow_control = False
    with pytest.raises(PermissionError):
        app.action(dict(action='switch', mode='joint', confirmed=True))
    with pytest.raises(ValueError, match='confirmation'):
        app.action(dict(action='switch', mode='joint'))


def test_preview_actions_are_local_and_recordings_not_replayable(app):
    app.action(dict(action='switch', mode='joint', confirmed=True))
    assert app.state.controllers[JOINT] == 'active'
    app.record_action('record_start')
    time.sleep(0.17)
    app.record_action('record_stop')
    files = app.configuration()['recordings']
    assert len(files) == 1 and files[0]['replay'] is None
    assert files[0]['name'].startswith('web_preview_')
    app.backend.targets['left_joint_0'] = 0.3
    app.action(dict(action='zero', side='right', duration=8, confirmed=True))
    assert app.backend.targets['left_joint_0'] == 0.3
    assert all(q == 0 for n, q in app.backend.targets.items() if n.startswith('right'))


def test_http_csrf_host_and_path_boundaries(app):
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(app))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    origin = f'http://127.0.0.1:{server.server_port}'
    try:
        with urllib.request.urlopen(origin + '/api/bootstrap') as response:
            assert json.load(response)['preview'] is True
        with urllib.request.urlopen(origin + '/api/pose-stream', timeout=2) as response:
            assert response.headers['Content-Type'] == 'text/event-stream'
            for _ in range(3):
                line = response.readline().decode()
                assert len(json.loads(line.removeprefix('data: '))['positions']) == 14
                assert response.readline() == b'\n'
        payload = json.dumps(dict(action='switch', mode='joint', confirmed=True)).encode()
        for headers in ({}, {'Origin': 'https://untrusted.example', 'X-Console-Token': app.token},
                        {'Origin': origin, 'X-Console-Token': 'invalid'}, {'Host': 'untrusted.example'}):
            request = urllib.request.Request(origin + '/api/action', data=payload,
                                             headers={'Content-Type': 'application/json', **headers})
            with pytest.raises(urllib.error.HTTPError) as exc:
                urllib.request.urlopen(request)
            assert exc.value.code == 403
        request = urllib.request.Request(origin + '/api/action', data=payload,
                                         headers={'Content-Type': 'application/json', 'Origin': origin,
                                                  'X-Console-Token': app.token})
        with urllib.request.urlopen(request) as response:
            assert 'Preview' in json.load(response)['message']
        for path in ('/%2e%2e/package.xml', '/mesh/../../../etc/passwd'):
            with pytest.raises(urllib.error.HTTPError) as exc:
                urllib.request.urlopen(origin + path)
            assert exc.value.code in (403, 404)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_live_adapter_has_no_process_or_raw_command_entrypoint():
    source = (ROOT / 'ieir_bringup/scripts/web_console_ros.py').read_text()
    assert 'subprocess' not in source and 'MotorCommand' not in source and 'EnableMotor' not in source
    assert 'check_motor_feedback' in source and 'check_no_other_target_publisher' in source
