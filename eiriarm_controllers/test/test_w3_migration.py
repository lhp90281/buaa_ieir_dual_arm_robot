"""Offline regression checks. No ROS node, socket or motor is started."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import pytest
import yaml
from w3_robot_bridge.msg import MotorCommand, MotorState, MotorStateArray

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'eiriarm_controllers/scripts'))


def load(relative):
    path = ROOT / relative
    name = path.stem.replace('.', '_')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def test_gripper_model_changes_mass_not_arm_geometry():
    launch = load('eiriarm_controllers/launch/dual_arm.launch.py')
    xacro = str(ROOT / 'eiriarm_controllers/config/dual_arm_ros2_control.urdf.xacro')
    full_xml = launch.build_robot_description(xacro, '/tmp/test_offsets.yaml', 'dual', True)
    bare_xml = launch.build_robot_description(xacro, '/tmp/test_offsets.yaml', 'dual', False)
    bare = ET.fromstring(bare_xml)
    assert not any('gripper' in link.get('name') for link in bare.findall('link'))
    assert len(bare.find('ros2_control').findall('joint')) == 14
    full_model = pin.buildModelFromXML(full_xml)
    bare_model = pin.buildModelFromXML(bare_xml)
    full_data, bare_data = full_model.createData(), bare_model.createData()
    payload_mass = sum(float(link.find('inertial/mass').get('value'))
                       for link in ET.fromstring(full_xml).findall('link')
                       if 'gripper' in link.get('name'))
    assert payload_mass > 0
    assert pin.computeTotalMass(full_model) - pin.computeTotalMass(bare_model) == pytest.approx(payload_mass)
    largest_gravity_difference = 0.0
    for angle in (0.0, 0.3, -0.6):
        configs = []
        torques = []
        for model, data in ((full_model, full_data), (bare_model, bare_data)):
            q = pin.neutral(model)
            indices = []
            for side in ('left', 'right'):
                for i in range(7):
                    joint = model.joints[model.getJointId(f'{side}_joint_{i}')]
                    q[joint.idx_q] = angle
                    indices.append(joint.idx_v)
            configs.append(q)
            torques.append(pin.computeGeneralizedGravity(model, data, q)[indices])
            pin.framesForwardKinematics(model, data, q)
        largest_gravity_difference = max(largest_gravity_difference,
                                        np.linalg.norm(torques[0] - torques[1]))
        for side in ('left', 'right'):
            frame = f'{side}_attachment_point'
            np.testing.assert_allclose(
                full_data.oMf[full_model.getFrameId(frame)].homogeneous,
                bare_data.oMf[bare_model.getFrameId(frame)].homogeneous, atol=1e-12)
    assert largest_gravity_difference > 0.1
    print(f'Removed gripper mass: {payload_mass:.6f} kg; gravity change: '
          f'{largest_gravity_difference:.6f} Nm (vector norm)')


def test_bridge_is_dual_arm_only_and_gripper_is_opt_in():
    launch = load('W3_ROBOT/w3_robot_bridge/launch/w3_robot_bridge.launch.py')
    share = ROOT / 'W3_ROBOT/w3_robot_bridge'
    for enabled, count in ((False, 7), (True, 8)):
        cfg = launch.bridge_config(share, enabled, 'dual')
        assert cfg == launch.bridge_config(share, enabled)
        assert [ch['iface'] for ch in cfg['channels']] == ['can0', 'can1']
        assert all(ch['motor_count'] == count and len(ch['motor_configs']) == count
                   for ch in cfg['channels'])
    for side, channel in (('left', 0), ('right', 1)):
        for i, (model, velocity, torque) in enumerate([
                ('DM8009', 45, 54), ('DM8009', 45, 54), ('DM4340P', 20, 28),
                ('DM4340', 20, 28), *[('DM4310', 50, 10)] * 4], 1):
            cfg = yaml.safe_load((share / f'config/motor_config/{side}_arm/can{channel}_id{i}.yaml').read_text())
            assert cfg['motor_model'] == model
            assert cfg['can_id'] == i
            assert cfg['ranges']['velocity'] == {'min': -velocity, 'max': velocity}
            assert cfg['ranges']['torque'] == {'min': -torque, 'max': torque}
            assert cfg['limits']['position'] == {'min': -12.5, 'max': 12.5}
            assert cfg['position_offset'] == 0
            assert cfg['signs'] == {'position': 1, 'velocity': 1, 'torque': 1}


@pytest.mark.parametrize('arms', ['left', 'right'])
@pytest.mark.parametrize('gripper', [False, True])
def test_single_arm_bridge_preserves_channel_indices(arms, gripper):
    launch = load('W3_ROBOT/w3_robot_bridge/launch/w3_robot_bridge.launch.py')
    share = ROOT / 'W3_ROBOT/w3_robot_bridge'
    cfg = launch.bridge_config(share, gripper, arms)
    active = 0 if arms == 'left' else 1
    assert len(cfg['channels']) == 2
    assert cfg['channels'][1-active] == {
        'iface': '', 'motor_count': 0, 'motor_configs': []}
    channel = cfg['channels'][active]
    assert channel['iface'] == f'can{active}'
    assert channel['motor_count'] == (8 if gripper else 7)
    assert len(channel['motor_configs']) == channel['motor_count']
    assert all(f'/{arms}_arm/can{active}_id' in path
               for path in channel['motor_configs'])
    robot = load('eiriarm_controllers/launch/dual_arm.launch.py')
    xml = robot.build_robot_description(
        str(ROOT / 'eiriarm_controllers/config/dual_arm_ros2_control.urdf.xacro'),
        '/tmp/test_offsets.yaml', arms, gripper)
    joints = ET.fromstring(xml).find('ros2_control').findall('joint')
    assert [j.get('name') for j in joints] == [f'{arms}_joint_{i}' for i in range(7)]
    assert all(j.find("param[@name='channel']").text == str(active) for j in joints)


class Sink:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_calibration_filters_channels_offline_and_nonfinite():
    mod = load('eiriarm_controllers/scripts/joint_zero_calibration.py')
    node = object.__new__(mod.CalibrationNode)
    node.channel = 1
    node.joints = [SimpleNamespace(slot=0)]
    node._state_lock = threading.Lock()
    node._states, node._state_err, node._state_received_at = {}, {}, {}
    node._sampling_active = True
    node._sample_buf = {0: []}
    msg = MotorStateArray()
    msg.header.stamp.sec = 42
    msg.motors = [
        MotorState(channel=0, motor_index=0, online=True, position=9.0),
        MotorState(channel=1, motor_index=0, online=True, position=0.25, error_flags=1),
    ]
    node._on_state(msg)
    assert node._states[0][0] == 0.25
    assert node._snapshot_err()[0] == 1
    assert node._snapshot_err(time.monotonic() + 1.0)[0] is None
    assert node._sample_buf[0] == [(42000000000, 0.25)]
    for offline, position in ((True, 10.0), (False, float('nan'))):
        msg.motors[1].online = not offline
        msg.motors[1].position = position
        node._on_state(msg)
        assert node._snapshot_err()[0] is None
    assert len(node._sample_buf[0]) == 1
    node._cmd_pub = Sink()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: msg.header.stamp))
    node._tick_cmd()
    node._publish_enable_msg(True)
    node._publish_enable_msg(False)
    assert [arr.commands[0].mode for arr in node._cmd_pub.messages] == [
        MotorCommand.MODE_RUN, MotorCommand.MODE_ENABLE, MotorCommand.MODE_DISABLE]
    assert all(arr.commands[0].channel == 1 and arr.commands[0].motor_index == 0
               for arr in node._cmd_pub.messages)


def test_calibration_no_data_cannot_confirm_disabled():
    mod = load('eiriarm_controllers/scripts/joint_zero_calibration.py')
    node = object.__new__(mod.CalibrationNode)
    node.channel = 1
    node.joints = [SimpleNamespace(slot=0)]
    node._state_lock = threading.Lock()
    node._state_err, node._state_received_at = {}, {}
    node._cmd_pub = Sink()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: MotorStateArray().header.stamp))
    logs = []
    node.get_logger = lambda: SimpleNamespace(info=logs.append, warn=logs.append)
    assert node.disable_all(timeout=0.025) is False
    assert any('NOT confirmed' in entry and 'no-data' in entry for entry in logs)
    assert any(arr.commands[0].mode == MotorCommand.MODE_DISABLE
               for arr in node._cmd_pub.messages)


def test_merge_uses_w3_channel_mapping():
    mod = load('ros2_ws_config/merge_offsets.py')
    sides = [{'channel': channel, 'offsets': [
        {'name': f'{side}_joint_{i}', 'slot': i, 'zero_offset': 0.1,
         'raw_at_reference': 0.6, 'urdf_pos_at_reference': 0.5}
        for i in range(7)]} for side, channel in (('left', 0), ('right', 1))]
    merged = mod.merge(*sides, {'offsets': [{'name': 'right_joint_0', 'axis_sign': -1}]})
    assert [e['channel'] for e in merged['offsets']] == [0] * 7 + [1] * 7
    assert merged['offsets'][7]['axis_sign'] == -1
    assert merged['offsets'][7]['zero_offset'] == pytest.approx(1.1)
    sides[1]['channel'] = 2
    with pytest.raises(ValueError):
        mod.merge(*sides, {})


def test_disabled_grippers_do_not_publish_teleop_commands():
    mod = load('eiriarm_controllers/scripts/teleop_joint_bridge.py')
    node = object.__new__(mod.TeleopJointBridge)
    node.args = SimpleNamespace(gripper='false')
    node.left_gripper_command_pub = Sink()
    node.right_gripper_command_pub = Sink()
    node._publish_gripper_command('left', 'teleop_passive')
    node._publish_gripper_command('right', 'hold')
    node._publish_gripper_targets()
    assert not node.left_gripper_command_pub.messages
    assert not node.right_gripper_command_pub.messages


def test_direction_preview_uses_only_delta_and_requires_motion():
    mod = load('eiriarm_controllers/scripts/calibration_direction.py')
    joints = [SimpleNamespace(name=f'right_joint_{i}', slot=i) for i in range(7)]
    check = mod.DirectionCheck(joints)
    check.update(8.3)
    assert check.preview() == ('right_joint_0', 0)
    check.key('enter', 8.3)
    check.key('enter', 8.3)
    assert check.index == 0
    check.update(8.5)
    assert check.preview()[1] == pytest.approx(0.2)
    check.key('space', 8.5)
    assert check.preview()[1] == pytest.approx(-0.2)
    check.key('enter', 8.5)
    assert check.preview() == ('right_joint_1', 0)
    assert check.signs[0] == -1
    check.key('backspace', 8.5)
    assert check.index == 0 and not check.confirmed[0]
    for i in range(7):
        check.key('enter', 5.0)
        check.update(5.15)
        check.key('enter', 5.15)
    assert check.done and len(check.results()) == 7


def test_direction_preview_blocks_dropouts_wraps_and_large_motion():
    mod = load('eiriarm_controllers/scripts/calibration_direction.py')
    for bad in (None, float('nan'), -12.4, 1.0):
        check = mod.DirectionCheck([SimpleNamespace(name='left_joint_0', slot=0)])
        check.key('enter', 0.0)
        check.update(0.1)
        check.update(bad)
        assert check.blocked
        check.key('enter', 0.1)
        assert not check.done
        check.key('r', 0.1)
        assert check.baseline is None and check.preview()[1] == 0
        with pytest.raises(ValueError):
            check.results()


def test_direction_file_and_signed_hard_stop_formula():
    mod = load('eiriarm_controllers/scripts/calibration_direction.py')
    zero = load('eiriarm_controllers/scripts/joint_zero_calibration.py')
    cfg = SimpleNamespace(channel=1, joints=[SimpleNamespace(name='right_joint_0', slot=0)])
    data = {'mode': 'direction', 'channel': 1, 'directions': [dict(
        name='right_joint_0', slot=0, axis_sign=-1, direction_verified=True)]}
    mod.load_directions(data, cfg)
    assert cfg.joints[0].axis_sign == -1
    for sign in (-1, 1):
        offset = zero.reference_offset(5.2, -1.5708, sign)
        assert sign * (5.2 - offset) == pytest.approx(-1.5708)
    data['channel'] = 0
    with pytest.raises(ValueError):
        mod.load_directions(data, cfg)


def test_merge_new_verified_sign_wins_over_old_file():
    mod = load('ros2_ws_config/merge_offsets.py')
    sides = [{'channel': ch, 'offsets': [dict(name=f'{side}_joint_{i}', slot=i,
              axis_sign=-1, direction_verified=True, zero_offset=2.3)
              for i in range(7)]} for side, ch in [('left', 0), ('right', 1)]]
    previous = {'offsets': [dict(name='right_joint_0', axis_sign=1)]}
    result = mod.merge(*sides, previous)
    assert result['offsets'][7]['axis_sign'] == -1
    assert result['offsets'][7]['zero_offset'] == 2.3


def test_hard_stop_capture_keeps_verified_negative_direction(monkeypatch, tmp_path):
    mod = load('eiriarm_controllers/scripts/joint_zero_calibration.py')
    joint = SimpleNamespace(name='right_joint_3', slot=3, motor_type='DM4340',
                            limit_side='negative', urdf_pos_at_limit=-1.5708,
                            axis_sign=-1, direction_verified=True,
                            reference_original=-1.5708, reference_confirmed=True)
    cfg = SimpleNamespace(window_sec=0, min_samples=3, max_motion_rad=0.05)
    node = SimpleNamespace(get_state=lambda slot: (4.0, 0, 0),
                           begin_sampling=lambda slot: None,
                           end_sampling=lambda slot: [(1, 4.0), (2, 4.0), (3, 4.0)])
    monkeypatch.setattr(mod, '_wait_enter', lambda *args: True)
    monkeypatch.setattr(mod, '_read_key', lambda *args: 'enter')
    result = mod._calibrate_one(node, joint, cfg, threading.Event())
    assert result['axis_sign'] == -1 and result['direction_verified']
    assert result['zero_offset'] == pytest.approx(2.4292)
    path = tmp_path / 'offsets.yaml'
    mod._write_offsets_yaml(path, 1, 'hard-stop', [result])
    entry = yaml.safe_load(path.read_text())['offsets'][0]
    assert entry['axis_sign'] * (4.0 - entry['zero_offset']) == pytest.approx(-1.5708)


def test_limit_review_stages_angles_in_requested_order():
    mod = load('eiriarm_controllers/scripts/calibration_direction.py')
    joints = [SimpleNamespace(name=f'right_joint_{i}', slot=i, urdf_pos_at_limit=-1.5)
              for i in range(7)]
    review = mod.LimitReview(joints, {})
    assert [j.slot + 1 for j in review.joints] == [7, 6, 5, 3, 4, 2, 1]
    review.key('angle_deg:-85')
    assert review.angles[0] == pytest.approx(math.radians(-85))
    assert joints[6].urdf_pos_at_limit == -1.5
    with pytest.raises(ValueError):
        review.apply()
    for _ in joints:
        review.key('enter')
    review.apply()
    assert all(j.reference_confirmed for j in joints)
    assert joints[6].reference_original == -1.5
    assert joints[6].urdf_pos_at_limit == pytest.approx(math.radians(-85))


@pytest.mark.parametrize('value', ['nan', 'inf', '361', '-361', '', 'abc'])
def test_limit_review_blocks_invalid_edits(value):
    mod = load('eiriarm_controllers/scripts/calibration_direction.py')
    joint = SimpleNamespace(name='right_joint_6', slot=6, urdf_pos_at_limit=-1.5)
    review = mod.LimitReview([joint], {})
    review.key('angle_deg:' + value)
    review.key('enter')
    assert not review.done
    assert review.angles == [-1.5]
    review.key('r')
    review.key('enter')
    assert review.done


def test_limit_review_backtrack_and_signed_offset():
    mod = load('eiriarm_controllers/scripts/calibration_direction.py')
    zero = load('eiriarm_controllers/scripts/joint_zero_calibration.py')
    joints = [SimpleNamespace(name=f'right_joint_{i}', slot=i, urdf_pos_at_limit=-1.5)
              for i in (5, 6)]
    review = mod.LimitReview(joints, {})
    review.key('right_fine')
    review.key('enter')
    review.key('backspace')
    assert not any(review.confirmed)
    review.key('angle_deg:-90')
    review.key('enter')
    review.key('enter')
    review.apply()
    offset = zero.reference_offset(4.0, joints[1].urdf_pos_at_limit, -1)
    assert -(4.0 - offset) == pytest.approx(-math.pi / 2)


@pytest.mark.parametrize('mode,accepted,expected', [
    ('limit-preview', True, 0), ('limit-preview', False, 130), ('hard-stop', False, 130)])
def test_limit_gate_does_not_construct_motor_node(monkeypatch, tmp_path, mode, accepted, expected):
    mod = load('eiriarm_controllers/scripts/joint_zero_calibration.py')
    monkeypatch.setattr(mod.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(mod.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(mod.signal, 'signal', lambda *args: None)
    monkeypatch.setattr(mod, 'review_limit_references', lambda *args: accepted)

    def forbidden(*args, **kwargs):
        pytest.fail('Preview/cancel must not create hardware command publishers')

    monkeypatch.setattr(mod, 'CalibrationNode', forbidden)
    assert mod.main(['--mode', mode, '--calibration-yaml',
                     str(ROOT / 'ros2_ws_config/joint_calibration_dual_right.yaml'),
                     '--output', str(tmp_path / 'reviewed.yaml')]) == expected
