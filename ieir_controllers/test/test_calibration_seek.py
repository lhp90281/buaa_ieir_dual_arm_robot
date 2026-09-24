"""Offline state-machine and file checks; never publishes ROS motor commands."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml
from calibration_test_data import write_test_inputs

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ieir_controllers/scripts'))
from calibration_seek import FrictionModel, PDCommand, SeekLimits, SeekMachine
from joint_auto_calibration import SingleJointNode, load_controller_gains, prepare, save_result
import joint_auto_calibration as runner


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('limits', [SeekLimits()] + [
    SeekLimits(kp=10, kd=kd, max_position_error=0.06) for kd in (0.7, 0.6, 0.5)])
def test_full_seek_zero_repeat_return_verify(sign, limits):
    machine = SeekMachine(sign, -1.0, 'negative', limits)
    q, v, effort = 2.0, 0.0, 0.0
    stop_q = 2.0 - sign
    states = set()
    for step in range(22000):
        now = step * 0.01
        states.add(machine.state)
        key = 'enter' if machine.state in ('ready', 'contact', 'return_confirm') else None
        if machine.state == 'verify':
            q += 0.0005
            v = 0.05
            effort = 0.0
            key = 'enter' if machine.verify_motion > 0.09 else None
        command = machine.update(now, (q, v, effort), key)
        assert command.torque == 0
        if command.active:
            assert abs(command.position - q) <= machine.limits.max_position_error + 1e-9
            assert abs(command.velocity) <= machine.limits.speed
        assert machine.state != 'fault', (machine.reason, q, v, states)
        if machine.state == 'done':
            break
        if machine.state != 'verify':
            for _ in range(10):
                effort = command.kp * (command.position - q) + command.kd * (command.velocity - v)
                v += (effort - 0.4 * v) / 0.05 * 0.001
                q += v * 0.001
                if machine.direction * (q - stop_q) > 0:
                    q, v = stop_q, 0.0
    assert machine.state == 'done', (machine.state, q, v)
    assert {'seek', 'contact', 'return_probe', 'reseek', 'return_confirm', 'return', 'verify'} <= states
    assert machine.offset == pytest.approx(2.0, abs=0.015)
    assert sign * (machine.stop_raw - machine.offset) == pytest.approx(-1.0)


@pytest.mark.parametrize('sample,dt,reason', [
    (None, 0.01, 'feedback'), ((math.nan, 0, 0), 0.01, 'feedback'),
    ((0.1, 0, 0), 0.01, 'encoder'), ((0, 0.3, 0), 0.01, 'overspeed'),
    ((0, 0, 1.1), 0.01, 'torque'), ((0, 0, 0), 0.3, 'control_loop')])
def test_faults_latch_zero_output(sample, dt, reason):
    machine = SeekMachine(1, -1, 'negative', SeekLimits())
    machine.update(0, (0, 0, 0), 'enter')
    assert machine.update(dt, sample) == PDCommand()
    assert machine.state == 'fault' and reason in machine.reason
    assert machine.update(dt + 0.01, (0, 0, 0), 'enter') == PDCommand()
    assert machine.state == 'fault'


def test_initial_obstruction_is_not_saved_as_stop():
    machine = SeekMachine(1, -1, 'negative', SeekLimits())
    machine.update(0, (0, 0, 0), 'enter')
    for step in range(1, 200):
        machine.update(step * 0.01, (0, -0.0122, -0.5))
    assert machine.state == 'initial_stall'
    assert machine.offset is None and machine.command == PDCommand()
    machine.update(2.0, (0.001, 0, 0), 'r')
    assert machine.state == 'ready' and machine.first_stop is None and machine.offset is None
    machine.update(2.01, (0.001, 0, 0), 'enter')
    assert machine.state == 'seek' and machine.phase_q == 0.001


def test_initial_stall_requires_explicit_confirmation_and_full_reprobe():
    m = SeekMachine(1, -1, 'negative', SeekLimits())
    m.update(0, (2, 0, 0), 'enter')
    for step in range(1, 200):
        m.update(step * 0.01, (2, 0, -0.55))
    assert m.state == 'initial_stall' and m.offset is None
    command = m.update(2.0, (2, 0, 0), 'enter')
    assert m.state == 'return_probe' and m.goal == 3 and m.offset is None
    assert command.position > 2
    m.start('reseek', 2.01, 3)
    m.last_q, m.last_t = 3, 2.01
    for step in range(1, 150):
        m.update(2.01 + step * 0.01, (3, 0, -0.55))
    assert m.state == 'fault' and m.reason == 'reseek_insufficient_motion'
    assert m.offset is None


@pytest.mark.parametrize('state,next_state', [('contact', 'return_probe'),
                                           ('initial_stall', 'return_probe'),
                                           ('return_confirm', 'return')])
@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('rebound', [0.0152587890625, 0.03])
def test_passive_rebound_does_not_revalidate_or_move_captured_stop(state, next_state, sign, rebound):
    limits = SeekLimits(kp=10, kd=0.6, stall_torque=1, effort_abort=1.5,
                        max_position_error=0.12, speed=0.2699333333333333,
                        max_speed=0.4318933333333333, hard_max_speed=0.5398666666666666)
    m = SeekMachine(sign, -0.8098, 'negative', limits, return_duration=3)
    loaded_stop = 1.3200960159301758
    m.first_stop = loaded_stop
    if state == 'return_confirm':
        m._reference(loaded_stop)
    m.state = state
    offset_before = m.offset
    assert m.update(0, (loaded_stop, 0, 0)) == PDCommand()
    for i in range(1, 101):
        actual = loaded_stop - m.direction * rebound * min(i / 20, 1)
        assert m.update(i * 0.01, (actual, 0.01221, 0.0024)) == PDCommand()
        assert m.first_stop == loaded_stop and m.offset == offset_before
    command = m.update(1.01, (actual, -0.01221, -0.01221), 'enter')
    assert m.state == next_state
    assert m.first_stop == loaded_stop and m.offset == offset_before
    assert m.goal == pytest.approx(loaded_stop - sign * m.reference)
    assert m.phase_q == actual
    assert 0 < (command.position - actual) * -m.direction <= limits.speed * 0.01001
    assert command.kp == 10 and command.kd == 0.6


@pytest.mark.parametrize('direction', [-1, 1])
def test_encoder_jitter_at_lead_bound_has_no_velocity_feedforward(direction):
    m = SeekMachine(direction, 1, 'positive', SeekLimits())
    m.update(0, (2, 0, 0), 'enter')
    m.target = 2 + direction * m.limits.max_position_error
    command = m.update(0.01, (2 - direction * 0.00038147, 0.01221, 0))
    assert command.velocity == 0
    assert abs(command.position - (2 - direction * 0.00038147)) <= m.limits.max_position_error + 1e-9


def test_manual_never_drives_and_needs_no_second_confirmation():
    machine = SeekMachine(-1, 1.57, 'positive', SeekLimits(), manual=True)
    for step in range(120):
        assert machine.update(step * 0.01, (2, 0, 0)) == PDCommand()
    machine.update(1.2, (2, 0, 0), 'enter')
    assert machine.state == 'manual_preview'
    assert machine.offset == pytest.approx(3.57)
    machine.update(1.21, (2, 0, 0), 'enter')
    assert machine.state == 'manual_preview'
    for step in range(1, 101):
        assert machine.update(1.21 + step * 0.01, (2 + step * 0.001, 0.1, 0)) == PDCommand()
    machine.update(2.22, (2.1, 0, 0), 'n')
    assert machine.state == 'done'
    assert machine.first_stop is None and not machine.retreats


def test_direction_reference_conflict_fails_before_motion():
    with pytest.raises(ValueError, match='conflicts'):
        SeekMachine(1, 1.57, 'negative', SeekLimits())


def args_for(tmp_path, **kwargs):
    reference_path, direction_path = write_test_inputs(tmp_path)
    values = dict(calibration_yaml=reference_path,
                  directions_yaml=direction_path,
                  output=tmp_path / 'offsets.yaml', joint=None, manual=False, stall_torque=0.5, speed=0.025,
                  cal_kp=None, cal_kd=None, max_position_error=None, effort_abort=1.0,
                  gains_yaml=ROOT / 'ieir_controllers/config/teleop_joint_gains.yaml',
                  gain_profile='slave', max_speed=0.30, hard_max_speed=0.50, motion_duration=3.0,
                  target_tolerance=0.01, return_min_fraction=0.5, auto_group=False, manual_group=False,
                  friction_compensation=False,
                  friction_yaml=ROOT / 'ros2_ws_config/friction_model.yaml',
                  controller_config=ROOT / 'ieir_controllers/config/dual_arm_controllers.yaml')
    values.update(kwargs)
    return SimpleNamespace(**values)


def prepare_legacy_motion(args):
    """Offline regression for the old seek math; not a supported robot entry point."""
    cfg, joint, _, output, previous, text = prepare(SimpleNamespace(**{**vars(args), 'auto_group': False}))
    limits, source = runner.motion_limits(args, joint, False)
    limits.validate(joint.tor_max)
    friction, friction_source = runner.load_calibration_friction(args, joint, False)
    engine = SeekMachine(joint.axis_sign, joint.urdf_pos_at_limit, joint.limit_side, limits,
                         return_duration=args.motion_duration, friction=friction)
    engine.gain_source, engine.friction_source = source, friction_source
    return cfg, joint, engine, output, previous, text


def test_manual_reference_preflight_and_no_automatic_skip(tmp_path):
    cfg, joint, engine, output, previous, text = prepare(args_for(tmp_path))
    assert joint.slot == 6 and engine.state == 'manual' and cfg.channel == 1
    assert prepare(args_for(tmp_path, joint=3))[2].manual
    with pytest.raises(ValueError, match='limit-preview'):
        prepare(args_for(tmp_path, calibration_yaml=ROOT / 'ros2_ws_config/joint_calibration_dual_right.yaml'))
    engine.offset, engine.stop_raw = 2.0, 2.0 + joint.axis_sign * joint.urdf_pos_at_limit
    with pytest.raises(ValueError, match='unverified'):
        save_result(output, previous, text, joint, engine, cfg.channel)
    engine.state = 'done'
    save_result(output, previous, text, joint, engine, cfg.channel)
    assert prepare(args_for(tmp_path))[1].slot == 6
    data = yaml.safe_load(output.read_text())
    assert data['complete'] is False and len(data['offsets']) == 1
    with pytest.raises(RuntimeError, match='changed'):
        save_result(output, previous, text, joint, engine, cfg.channel)


def test_all_joints_are_always_manual_without_loading_gains_or_friction(tmp_path):
    args = args_for(tmp_path)
    source = yaml.safe_load(args.calibration_yaml.read_text())
    signs = yaml.safe_load(args.directions_yaml.read_text())['directions']
    args.output.write_text(yaml.safe_dump(dict(channel=1, offsets=[
        dict(slot=j['slot'], name=j['name'], axis_sign=signs[j['slot']]['axis_sign'], zero_offset=0.0)
        for j in source['joints']])))
    args.gains_yaml = args.friction_yaml = tmp_path / 'does-not-exist.yaml'
    args.cal_kp = 500
    args.friction_compensation = True
    for joint in range(1, 8):
        args.joint = joint
        m = prepare(args)[2]
        assert m.state == 'manual' and m.manual and m.friction == FrictionModel()
        for i in range(140):
            assert m.update(i * 0.01, (2, 0, 0), 'enter' if i == 120 else None) == PDCommand()
        assert m.state == 'manual_preview' and not m.retreats and m.first_stop is None


def test_manual_group_captures_saves_once_and_preserves_auto_results(tmp_path):
    args = args_for(tmp_path, manual_group=True)
    directions = yaml.safe_load(args.directions_yaml.read_text())['directions']
    original = [dict(slot=slot, name=f'right_joint_{slot}', axis_sign=directions[slot]['axis_sign'],
                     zero_offset=0.123, method='repeated_position_pd_seek') for slot in (6, 5, 4, 2)]
    args.output.write_text(yaml.safe_dump(dict(channel=1, offsets=original)))
    assert prepare(args)[1].slot == 6
    for number in (4, 2, 1):
        args.joint = number
        cfg, joint, m, output, previous, old_text = prepare(args)
        assert m.manual and m.state == 'manual' and m.friction == FrictionModel()
        for i in range(120):
            assert m.update(i * 0.01, (2, 0.01221, 0.1)) == PDCommand()
            assert runner.preview_angle(joint, m, 'active', (2, 0, 0), 1.5) == joint.urdf_pos_at_limit
        assert m.update(1.2, (2, 0.01221, 0.1), 'enter') == PDCommand()
        assert m.state == 'manual_preview'
        save_result(output, previous, old_text, joint, m, cfg.channel)
        saved = yaml.safe_load(output.read_text())
        assert all(entry in saved['offsets'] for entry in original)
        entry = next(e for e in saved['offsets'] if e['slot'] == number - 1)
        assert entry['method'] == 'manual' and entry['reference_confirmed']
        assert entry['visually_verified'] is False and not entry['retreats']
        assert runner.preview_angle(joint, m, 'active', (2.03, 0, 0), 1.5) == pytest.approx(
            joint.urdf_pos_at_limit + joint.axis_sign * 0.03)
        assert m.update(1.21, (2, 0, 0), 'n') == PDCommand()
        assert m.state == 'done', 'No extra confirmation or minimum hand travel required'
    assert saved['complete'] and len(saved['offsets']) == 7
    assert prepare(args_for(tmp_path, manual_group=True))[1].slot == 6
    with pytest.raises(ValueError, match='Automatic calibration is disabled'):
        prepare(args_for(tmp_path, auto_group=True))


@pytest.mark.parametrize('failure', [1, 2, None])
def test_manual_group_visits_all_seven_and_stops_on_failure(monkeypatch, failure):
    calls = []
    def run(args):
        calls.append(args.joint)
        assert args.manual_group and not args.auto_group
        return (failure or 0) if len(calls) == 2 else runner.SKIPPED
    monkeypatch.setattr(runner, 'run_single', run)
    result = runner.main(['--calibration-yaml', 'unused', '--directions-yaml', 'unused', '--manual-group'])
    assert calls == ([7, 6] if failure else [7, 6, 5, 3, 4, 2, 1])
    assert result == (failure or 0)


def test_group_options_are_mutually_exclusive():
    with pytest.raises(SystemExit):
        runner.main(['--calibration-yaml', 'unused', '--directions-yaml', 'unused',
                     '--auto-group', '--manual-group'])
    with pytest.raises(SystemExit):
        runner.main(['--calibration-yaml', 'unused', '--directions-yaml', 'unused', '--auto-group'])


def test_exit_after_manual_save_is_success_without_advancing_group(monkeypatch):
    calls = []
    def run(args):
        calls.append(args.joint)
        return runner.MANUAL_EXITED
    monkeypatch.setattr(runner, 'run_single', run)
    assert runner.main(['--calibration-yaml', 'unused', '--directions-yaml', 'unused', '--manual-group']) == 0
    assert calls == [7]


def test_manual_group_saves_all_seven_without_automatic_motion(tmp_path):
    args = args_for(tmp_path)
    for slot in (6, 5, 4, 2, 3, 1, 0):
        args.joint = slot + 1
        cfg, joint, engine, output, previous, text = prepare(args)
        assert joint.slot == slot and engine.state == 'manual'
        engine.stop_raw = 2 + joint.axis_sign * joint.urdf_pos_at_limit
        engine.offset, engine.state = 2, 'manual_preview'
        save_result(output, previous, text, joint, engine, cfg.channel)
    args.joint = None
    assert prepare(args)[1].slot == 6, 'Saved joint 7 must still be presented'
    data = yaml.safe_load(args.output.read_text())
    assert data['complete'] is True
    assert {e['slot'] for e in data['offsets']} == set(range(7))
    assert all(e['method'] == 'manual' for e in data['offsets'])


@pytest.mark.parametrize('failure', [1, 2, None])
def test_group_driver_stops_on_failure_or_completion_without_extra_joint(monkeypatch, failure):
    calls = []
    def run(args):
        calls.append(args)
        if len(calls) == 1:
            return runner.SKIPPED
        if failure is None:
            return 0
        return failure
    monkeypatch.setattr(runner, 'run_single', run)
    result = runner.main(['--calibration-yaml', 'unused.yaml', '--directions-yaml', 'unused.yaml'])
    assert [a.joint for a in calls] == ([7, 6] if failure else [7, 6, 5, 3, 4, 2, 1])
    assert result == (failure or 0)


def test_explicit_group_skip_can_leave_holes_without_fabricating_offsets(tmp_path):
    args = args_for(tmp_path, joint=3)
    assert prepare(args)[1].slot == 2
    assert not args.output.exists()


def test_nonrepeatable_stop_and_blocked_return():
    m = SeekMachine(1, -1, 'negative', SeekLimits())
    m.first_stop = -1
    m.start('reseek', 0, -0.8)
    for i in range(200):
        m.update(i * 0.01, (-0.9, 0, -0.5))
    assert m.state == 'fault' and m.reason == 'stop_not_repeatable'
    m = SeekMachine(1, -1, 'negative', SeekLimits())
    m.start('return', 0, -1, 0)
    for i in range(300):
        m.update(i * 0.01, (-1, 0, 0.5))
    assert m.state == 'fault' and 'blocked' in m.reason


@pytest.mark.parametrize('direction', [-1, 1])
def test_probe_return_does_not_accept_only_small_clearance(direction):
    m = SeekMachine(direction, 1, 'positive', SeekLimits())
    m.first_stop = 3.346876
    goal = m.first_stop - direction
    m.state = 'contact'
    m.update(0, (m.first_stop, 0, 0), 'enter')
    assert m.state == 'return_probe' and m.goal == goal and m.offset is None
    actual = m.first_stop - direction * 0.07172
    for i in range(1, 120):
        command = m.update(i * 0.01, (actual, -0.01221, 0.2369))
    assert m.state == 'return_probe' and m.offset is None
    m.start('return_probe', 2, goal, goal)
    m.last_q = goal
    m.last_t = 2
    for i in range(1, 120):
        command = m.update(2 + i * 0.01, (goal, -0.01221, 0))
        if m.state == 'reseek':
            break
    assert m.state == 'reseek'
    assert command == PDCommand()
    assert m.target == goal and m.offset is None
    assert m.phase_budget == m.limits.max_travel


@pytest.mark.parametrize('state,next_state', [('return_probe', 'reseek'), ('return', 'verify')])
def test_reported_static_zero_error_is_accepted_only_after_settling(state, next_state):
    goal, actual = 1.7760761444091797, 1.781682014465332
    m = SeekMachine(-1, -1.5708, 'negative', SeekLimits(), return_duration=3)
    m.start(state, 0, goal + 1.5708, goal)
    for i in range(120):
        m.update(i * 0.01, (actual, -0.012210845947265625, -0.051281929))
        if i < 100:
            assert m.state == state
        if m.state != state:
            break
    assert m.state == next_state
    assert m.limits.repeat_tolerance == 0.015


def test_retreat_does_not_complete_before_ramp_time_or_while_moving():
    for error, velocity in [(0.011, 0), (0.0056, 0.05)]:
        m = SeekMachine(1, 1, 'positive', SeekLimits())
        m.start('return_probe', 0, 2, 1)
        for i in range(120):
            m.update(i * 0.01, (1 + error, velocity, 0.05))
        assert m.state == 'return_probe'
    with pytest.raises(ValueError, match='target_tolerance'):
        SeekLimits(target_tolerance=0.03).validate(10)


@pytest.mark.parametrize('state,next_state', [('return_probe', 'reseek'), ('return', 'verify')])
def test_joint6_reported_return_error_does_not_require_exact_zero(state, next_state):
    goal, actual, start = 2.1295145462036134, 2.069695472717285, 1.3281068801879883
    limits = SeekLimits(kp=10, kd=0.6, stall_torque=1, effort_abort=1.5,
                        max_position_error=0.12, speed=0.2699333333333333,
                        max_speed=0.4318933333333333, hard_max_speed=0.5398666666666666)
    m = SeekMachine(1, -0.8098, 'negative', limits, return_duration=3)
    m.offset = goal if state == 'return' else None
    m.first_stop = goal - 0.8098
    m.start(state, 0, start, goal)
    for i in range(500):
        now = i * 0.01
        q = min(actual, start + m.phase_speed * now)
        command = m.update(now, (q, -0.01221, 0.6764345169))
        assert m.state != 'fault', m.reason
        if m.state == next_state:
            break
    assert m.state == next_state and command == PDCommand()
    assert m.retreats[-1]['completion'] == 'clearance'
    assert m.retreats[-1]['zero_error'] == pytest.approx(0.059819073486328)
    assert m.offset == (goal if state == 'return' else None)


@pytest.mark.parametrize('state,next_state', [('return_probe', 'reseek'), ('return', 'verify')])
@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('reference', [0.8098, 1.5708, 2.618])
def test_sufficient_retreat_accepts_static_error_in_both_directions(state, next_state, direction, reference):
    limits = SeekLimits(kp=10, kd=0.6, stall_torque=1, effort_abort=1.5,
                        max_position_error=0.12, speed=0.3, max_speed=0.5, hard_max_speed=0.6)
    m = SeekMachine(direction, reference, 'positive', limits, return_duration=3)
    goal = 3.0
    stop = goal + direction * reference
    m.offset = goal if state == 'return' else None
    m.first_stop = stop
    m.start(state, 0, stop, goal)
    for i in range(1500):
        now = i * 0.01
        travel = min(reference - 0.06, m.phase_speed * now)
        q = stop - direction * travel
        command = m.update(now, (q, -direction * 0.01221, -direction * 0.6))
        assert m.state != 'fault', m.reason
        if m.state == next_state:
            break
    assert m.state == next_state and command == PDCommand()
    result = m.retreats[-1]
    assert result['completion'] == 'clearance'
    assert result['clearance'] == pytest.approx(reference - 0.06)
    assert result['required'] == pytest.approx(reference / 2)
    assert m.offset == (goal if state == 'return' else None)
    if state == 'return_probe':
        assert m.target == q and m.phase_q == q


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('travel,velocity', [(0.1, 0), (-0.1, 0), (0.8, 0.05)])
def test_retreat_requires_large_signed_clearance_and_settling(direction, travel, velocity):
    m = SeekMachine(direction, 1, 'positive', SeekLimits(speed=0.3, max_speed=0.5, hard_max_speed=0.6))
    start, goal = 3 + direction, 3
    m.start('return_probe', 0, start, goal)
    for i in range(600):
        q = start - direction * min(abs(travel), 0.2 * i * 0.01) * (1 if travel > 0 else -1)
        m.update(i * 0.01, (q, velocity, 0.05))
    assert m.state == 'return_probe' and m.offset is None and not m.retreats


@pytest.mark.parametrize('fraction', [0, 0.09, 1.1, math.nan])
def test_invalid_retreat_fraction_rejected(fraction):
    with pytest.raises(ValueError):
        SeekLimits(return_min_fraction=fraction).validate(10)


def test_hard_overspeed_still_latches_with_exact_sample():
    m = SeekMachine(1, 1, 'positive', SeekLimits())
    m.update(0, (0, 0, 0), 'enter')
    previous = m.command
    sample = (0.001, 0.3, 0.15)
    assert m.update(0.01, sample) == PDCommand()
    assert m.state == 'fault' and m.reason == 'overspeed_hard'
    assert m.fault_snapshot['sample'] == sample
    assert m.fault_snapshot['previous_command']['position'] == previous.position
    assert m.fault_snapshot['previous_command']['kp'] == previous.kp
    assert m.limits.max_speed == 0.15


@pytest.mark.parametrize('direction', [-1, 1])
def test_hand_verification_is_passive_even_above_automatic_speed_guard(direction):
    m = SeekMachine(1, 1, 'positive', SeekLimits())
    m.state, m.verify_origin = 'verify', 1.781682014465332
    for i in range(12):
        command = m.update(i * 0.01, (m.verify_origin + direction * i * 0.01,
                                     direction * 1.1111106872558594, -0.007326))
        assert m.state == 'verify' and command == PDCommand()
    assert m.verify_motion >= 0.08
    assert m.update(0.12, (m.verify_origin + direction * 0.11, 0, 0), 'enter') == PDCommand()
    assert m.state == 'done'


@pytest.mark.parametrize('state', ['ready', 'contact', 'return_confirm'])
def test_starting_automatic_motion_still_rejects_excessive_hand_speed(state):
    m = SeekMachine(1, 1, 'positive', SeekLimits())
    m.state = state
    assert m.update(0, (2, 1.11, 0), 'enter') == PDCommand()
    assert m.state == 'fault' and m.reason == 'overspeed_hard'


def test_sustained_velocity_fault_with_stationary_encoder_is_reported_separately():
    m = SeekMachine(1, 1, 'positive', SeekLimits())
    m.update(0, (0, 0, 0), 'enter')
    for step in range(1, 13):
        command = m.update(step * 0.01, (0, 0.1587, 0.15))
    assert command == PDCommand()
    assert m.fault_snapshot['position_velocity'] == 0
    assert m.fault_snapshot['position_delta'] == 0
    assert m.fault_snapshot['sample'][1] == 0.1587
    assert m.state == 'fault' and m.reason == 'overspeed_sustained'


@pytest.mark.parametrize('direction', [-1, 1])
def test_short_speed_excursion_brakes_then_resumes_without_stored_position_error(direction):
    m = SeekMachine(1, 1, 'positive', SeekLimits())
    m.update(0, (2, 0, 0), 'enter')
    for step in range(1, 5):
        q = 2 + direction * step * 0.002
        command = m.update(step * 0.01, (q, direction * 0.2, 0.15))
        assert m.state == 'seek'
        assert command.position == q and command.velocity == 0 and command.torque == 0
        assert command.kd == m.limits.kd
    command = m.update(0.05, (q, 0, 0))
    assert m.overspeed_since is None
    assert command.position == pytest.approx(q + m.limits.speed * 0.01)
    assert command.velocity == pytest.approx(m.limits.speed)


@pytest.mark.parametrize('sign', [-1, 1])
def test_position_target_starts_at_feedback_and_cannot_wind_up(sign):
    m = SeekMachine(sign, 1, 'positive', SeekLimits())
    command = m.update(0, (2.0, 0, 0), 'enter')
    assert command.position == pytest.approx(2.0 + sign * 0.025 * 0.01)
    for step in range(1, 2000):
        command = m.update(step * 0.01, (2.0, 0.01221, 0))
        assert abs(command.position - 2) <= 0.2 + 1e-9
        assert command.kp == 3 and command.kd == 1 and command.torque == 0
    assert m.state == 'seek'
    assert command.position == pytest.approx(2 + sign * 0.2)
    assert command.velocity == 0


def test_stall_requires_full_point_five_and_reprobe_resets_target():
    m = SeekMachine(1, 1, 'positive', SeekLimits())
    m.update(0, (0, 0, 0), 'enter')
    for step in range(1, 101):
        m.update(step * 0.01, (step * 0.0005, 0.05, 0.4))
    for step in range(101, 250):
        m.update(step * 0.01, (0.05, 0.01221, 0.4))
    assert m.state == 'seek', '0.4 Nm must not count as the 0.5 Nm contact threshold'
    for step in range(250, 370):
        command = m.update(step * 0.01, (0.05, 0.01221, 0.55))
    assert m.state == 'contact' and command == PDCommand()
    command = m.update(3.7, (0.05, 0, 0), 'enter')
    assert m.state == 'return_probe'
    assert command.position == pytest.approx(0.05 - 0.025 * 0.01)


def test_native_pd_with_static_friction_has_no_host_torque_integrator():
    m = SeekMachine(1, 1, 'positive', SeekLimits())
    q = v = effort = 0.0
    peak_speed = 0.0
    for step in range(1200):
        command = m.update(step * 0.01, (q, v, effort), 'enter' if step == 0 else None)
        assert m.state != 'fault', m.reason
        assert command.torque == 0
        for _ in range(20):
            effort = command.kp * (command.position - q) + command.kd * (command.velocity - v)
            if abs(v) < 1e-4 and abs(effort) < 0.11:
                v = 0.0
            else:
                friction = math.copysign(0.03, v if abs(v) > 1e-4 else effort)
                next_v = v + (effort - friction) / 0.001 * 0.0005
                v = 0 if next_v * v < 0 and abs(effort) < 0.11 else next_v
            q += v * 0.0005
            peak_speed = max(peak_speed, abs(v))
    assert q > 0.1 and peak_speed < m.limits.max_speed


def test_pd_transport_and_passive_stop_packets():
    from builtin_interfaces.msg import Time
    messages = []
    node = SimpleNamespace(channel=1, joint=SimpleNamespace(slot=6, pos_max=12.5), limits=SeekLimits(), friction=FrictionModel(),
        fresh=lambda: (2, 0, 0), publisher=SimpleNamespace(publish=messages.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)))
    cmd = PDCommand(position=2.01, velocity=0.025, kp=3, kd=1)
    SingleJointNode.send(node, cmd)
    sent = messages[-1].commands[0]
    assert (sent.channel, sent.motor_index) == (1, 6)
    assert (sent.kp, sent.kd, sent.torque) == (3, 1, 0)
    assert sent.position == pytest.approx(2.01)
    SingleJointNode.send(node)
    sent = messages[-1].commands[0]
    assert sent.kp == sent.kd == sent.torque == sent.velocity == 0
    for bad in (PDCommand(position=2.3, kp=3, kd=1), PDCommand(torque=0.1),
                PDCommand(position=2, kp=9, kd=1), PDCommand(position=math.nan, kp=3, kd=1)):
        with pytest.raises(ValueError):
            SingleJointNode.send(node, bad)
    node.fresh = lambda: None
    with pytest.raises(ValueError):
        SingleJointNode.send(node, cmd)
    assert len(messages) == 2
    node.manual_only = True
    node.fresh = lambda: (2, 0, 0)
    for bad in (cmd, PDCommand(kd=0.1), PDCommand(velocity=0.01), PDCommand(torque=0.01)):
        with pytest.raises(ValueError, match='Manual-only'):
            SingleJointNode.send(node, bad)
    SingleJointNode.send(node)
    assert len(messages) == 3


def test_pd_settings_reject_unreachable_or_unsafe_stall_budget():
    with pytest.raises(ValueError, match='cannot reach'):
        SeekLimits(max_position_error=0.1).validate(10)
    with pytest.raises(ValueError, match='estimate'):
        SeekLimits(kp=10, kd=3, max_position_error=0.095).validate(10)
    SeekLimits().validate(10)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('slot,kd', [(4, 0.7), (5, 0.6), (6, 0.5)])
def test_production_wrist_gains_match_both_controller_sources(side, slot, kd):
    config = ROOT / 'ieir_controllers/config'
    name = f'{side}_joint_{slot}'
    for filename in ('teleop_joint_gains.yaml', 'dual_arm_controllers.yaml'):
        assert load_controller_gains(config / filename, name, 'slave') == (10, kd)


def test_legacy_motion_uses_controller_gains_with_smaller_target_lead(tmp_path):
    engine = prepare_legacy_motion(args_for(tmp_path))[2]
    assert (engine.limits.kp, engine.limits.kd) == (10, 0.5)
    assert engine.limits.max_position_error == pytest.approx(0.06)
    assert 'teleop_joint_gains.yaml' in engine.gain_source
    for i in range(1000):
        command = engine.update(i * 0.01, (2, 0, 0), 'enter' if i == 0 else None)
        assert command.kp == 10 and command.kd == 0.5 and command.torque == 0
        assert abs(command.position - 2) <= 0.06000001
    with pytest.raises(ValueError, match='estimate'):
        prepare_legacy_motion(args_for(tmp_path, max_position_error=0.2))
    with pytest.raises(ValueError, match='together'):
        prepare_legacy_motion(args_for(tmp_path, cal_kp=3))


def test_production_speed_guard_accepts_reported_brief_excursion(tmp_path):
    m = prepare_legacy_motion(args_for(tmp_path))[2]
    m.update(0, (1.791600227355957, 0, 0), 'enter')
    command = m.update(0.006126361, (1.7942705154418945, 0.3052520751953125, 0.11965847))
    assert m.state == 'seek' and command.velocity == 0
    assert command.position == pytest.approx(1.7942705154418945)
    m.update(0.02, (1.795, 0.1, 0.1))
    assert m.state == 'seek' and m.overspeed_since is None
    assert m.update(0.03, (1.796, 0.5, 0.1)) == PDCommand()
    assert m.reason == 'overspeed_hard'
    with pytest.raises(ValueError):
        prepare_legacy_motion(args_for(tmp_path, hard_max_speed=2.1))


def test_three_second_defaults_scale_speed_and_guards(tmp_path):
    _, joint, m, *_ = prepare_legacy_motion(args_for(tmp_path, speed=None, max_speed=None, hard_max_speed=None))
    speed = abs(joint.urdf_pos_at_limit) / 3
    assert m.limits.speed == pytest.approx(speed)
    assert m.limits.max_speed == pytest.approx(max(0.3, 1.6 * speed))
    assert m.limits.hard_max_speed == pytest.approx(max(0.5, 2 * speed))
    assert m.return_duration == 3
    m.start('return', 0, 2, 2 - abs(joint.urdf_pos_at_limit))
    assert m.phase_speed == pytest.approx(speed)
    m.start('return_probe', 0, 2, 1.95)
    assert m.phase_speed == pytest.approx(0.05 / 3)


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('reference,kd', [(1.5708, 0.5), (0.8098, 0.6), (2.618, 0.7)])
def test_fast_wrist_seek_and_return_reference_ramps(sign, reference, kd):
    speed = reference / 3
    limits = SeekLimits(kp=10, kd=kd, max_position_error=0.06, speed=speed,
                        max_speed=max(0.3, 1.6 * speed), hard_max_speed=max(0.5, 2 * speed))
    limits.validate(10)
    m = SeekMachine(sign, reference, 'positive', limits, return_duration=3)
    m.start('seek', 0, 2)
    for i in range(301):
        command = m.update(i / 100, (2 + sign * speed * i / 100, sign * speed, 0.1))
        assert m.state == 'seek'
        assert abs(command.velocity) * kd <= 0.200000001
    assert abs(command.position - 2) == pytest.approx(reference, abs=0.01)
    q0 = 2 + sign * reference
    m.start('return', 3.01, q0, 2)
    for i in range(301):
        command = m.update(3.01 + i / 100, (q0 - sign * speed * i / 100, -sign * speed, 0.1))
        assert m.state == 'return'
    assert command.position == pytest.approx(2, abs=0.01)


def test_gains_are_selected_by_name_and_invalid_files_fail_closed(tmp_path):
    path = tmp_path / 'gains.yaml'
    data = dict(joints=['right_joint_6', 'left_joint_6'],
                profiles=dict(slave=dict(kp_gains=[9, 10], kd_gains=[0.4, 0.5])))
    path.write_text(yaml.safe_dump(data))
    engine = prepare_legacy_motion(args_for(tmp_path, gains_yaml=path))[2]
    assert (engine.limits.kp, engine.limits.kd) == (9, 0.4)
    assert engine.limits.max_position_error == pytest.approx(0.6 / 9)
    with pytest.raises(OSError):
        prepare_legacy_motion(args_for(tmp_path, gains_yaml=tmp_path / 'missing.yaml'))
    data['profiles']['slave']['kd_gains'] = [0.4]
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='array lengths'):
        prepare_legacy_motion(args_for(tmp_path, gains_yaml=path))
    data['profiles']['slave']['kd_gains'] = [math.nan, 0.5]
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='finite'):
        prepare_legacy_motion(args_for(tmp_path, gains_yaml=path))


def test_real_friction_mapping_and_joint_three_abort_budget(tmp_path):
    args = args_for(tmp_path, auto_group=True, joint=6, friction_compensation=True,
                    speed=None, max_speed=None, hard_max_speed=None, effort_abort=None)
    m = prepare_legacy_motion(args)[2]
    assert m.friction.gain == 0.8 and m.friction.coulomb_neg == 0.099
    assert m.friction.torque(-m.limits.speed) == pytest.approx(-0.079947175, abs=1e-8)
    assert m.limits.effort_abort == 1
    args.joint = 3
    m = prepare_legacy_motion(args)[2]
    assert m.friction.gain == 0.85 and m.limits.effort_abort == 1.5
    args.effort_abort = 1.0
    with pytest.raises(ValueError, match='plus friction'):
        prepare_legacy_motion(args)
    args.joint = 6
    args.friction_yaml = tmp_path / 'missing.yaml'
    with pytest.raises(OSError):
        prepare_legacy_motion(args)


@pytest.mark.parametrize('direction', [-1, 1])
def test_contact_uses_feedback_minus_previous_ff_and_passive_is_zero(direction):
    friction = FrictionModel(0.0934, 0.099, 0.00346, 0.8)
    m = SeekMachine(direction, 1, 'positive', SeekLimits(), friction=friction)
    command = m.update(0, (2, 0, 0), 'enter')
    assert direction * command.torque > 0
    for i in range(1, 260):
        q = 2 + direction * min(i, 100) * 0.0005
        effort = command.torque + direction * 0.45
        command = m.update(i * 0.01, (q, direction * 0.01221, effort))
    assert m.state == 'seek', 'Raw effort >0.5 is insufficient after subtracting friction'
    for i in range(260, 390):
        command = m.update(i * 0.01, (q, 0, command.torque + direction * 0.55))
        if m.state == 'contact':
            break
    assert m.state == 'contact' and command == PDCommand()
    assert m.update(i * 0.01 + 0.01, (q, 0, 0)) == PDCommand()


def test_friction_never_masks_total_measured_torque_abort():
    m = SeekMachine(1, 1, 'positive', SeekLimits(), friction=FrictionModel(0.1, 0.1, 0, 1))
    m.update(0, (2, 0, 0), 'enter')
    assert m.update(0.01, (2, 0, 1.01)) == PDCommand()
    assert m.reason == 'measured_torque_limit'


def test_joint_six_raised_threshold_is_scoped_and_budgeted(tmp_path):
    args = args_for(tmp_path, auto_group=True, joint=6, friction_compensation=True,
                    speed=None, max_speed=None, hard_max_speed=None,
                    stall_torque=None, effort_abort=None)
    m = prepare_legacy_motion(args)[2]
    assert m.limits.stall_torque == 1 and m.limits.effort_abort == 1.5
    assert m.limits.max_position_error == pytest.approx(0.12)
    m.update(0, (2.15, 0, 0), 'enter')
    for i in range(1, 160):
        m.update(i * 0.01, (2.1345, -0.01221, -0.67155))
    assert m.state == 'seek', 'Previous 0.59 Nm residual must not trigger the new contact threshold'
    assert m.update(1.60, (2.1345, 0, -1.51)) == PDCommand()
    assert m.reason == 'measured_torque_limit'
    for joint, abort in [(7, 1), (5, 1), (3, 1.5)]:
        args.joint = joint
        other = prepare_legacy_motion(args)[2]
        assert other.limits.stall_torque == 0.5 and other.limits.effort_abort == abort
    args.joint, args.effort_abort = 6, 1.0
    with pytest.raises(ValueError):
        prepare_legacy_motion(args)
