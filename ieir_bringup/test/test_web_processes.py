"""Process ownership tests with fake children; no ROS or CAN."""
from pathlib import Path
import signal
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ieir_bringup/scripts'))
from web_console_core import ConsoleState, robot_model
from web_console_processes import ProcessManager


@pytest.fixture
def manager(tmp_path):
    directory = tmp_path / 'src/ros2_ws_config'
    directory.mkdir(parents=True)
    entries = []
    for side, channel in [('left', 0), ('right', 1)]:
        arm = [dict(name=f'{side}_joint_{i}', slot=i, channel=channel, axis_sign=1,
                    zero_offset=0.1) for i in range(7)]
        entries.extend(arm)
        (directory / f'joint_offsets_{side}.yaml').write_text(yaml.safe_dump(dict(offsets=arm)))
        for suffix in ('', '_reviewed'):
            (directory / f'joint_calibration_dual_{side}{suffix}.yaml').write_text(yaml.safe_dump(dict(channel=channel)))
        (directory / f'joint_directions_{side}.yaml').write_text('directions: []\n')
    (directory / 'joint_offsets_dual.yaml').write_text(yaml.safe_dump(dict(offsets=entries)))
    (directory / 'friction_model.yaml').write_text('{}\n')
    (directory / 'merge_offsets.py').write_text('# fake\n')
    model, _ = robot_model(ROOT / 'description')
    backend = SimpleNamespace(check_idle_graph=Mock(), check_target_start=Mock())
    item = ProcessManager(tmp_path, ConsoleState(model), backend)
    yield item
    # Test children are never real processes.
    item.jobs.clear()
    item.close()


def test_control_argv_is_fixed_and_does_not_recurse(manager):
    kind, argv, outputs, label = manager.plan(dict(kind='control', arms='dual'))
    assert kind == 'control' and not outputs
    assert 'use_web:=false' in argv and 'gripper:=false' in argv and 'controller:=gravity' in argv
    assert not any('bridge.launch' in arg or 'mujoco' in arg for arg in argv)
    assert any(str(manager.directory / 'joint_offsets_dual.yaml') in arg for arg in argv)


@pytest.mark.parametrize('data', [dict(kind='bridge'), dict(kind='shell', command='echo test'),
    dict(kind='control', arms='dual; echo test'), dict(kind='calibration', side='../../etc', stage='manual'),
    dict(kind='calibration', side='left', stage='seek'), dict(kind='calibration', side='left', stage='manual', joint=True),
    dict(kind='teleop', role='slave', peer_host='1.2.3.4;echo test', local_port=15000, peer_port=15001),
    dict(kind='replay', file='../data.yaml')])
def test_rejects_unapproved_commands_and_arguments(manager, data):
    with pytest.raises(ValueError):
        manager.plan(data)


@pytest.mark.parametrize('stage', ['direction', 'limits', 'manual'])
def test_calibration_only_uses_headless_existing_manual_workflows(manager, stage):
    _, argv, outputs, _ = manager.plan(dict(kind='calibration', side='right', stage=stage, joint=4))
    assert argv[-2:] == ['--preview-backend', 'web']
    assert len(outputs) == 1
    assert '--auto-group' not in argv
    if stage == 'manual':
        assert 'joint_manual_calibration' in argv and '--joint' in argv


def test_requires_complete_valid_local_offsets(manager):
    path = manager.directory / 'joint_offsets_dual.yaml'
    path.write_text('offsets: []\n')
    with pytest.raises(ValueError, match='Complete calibration'):
        manager.plan(dict(kind='control'))


def test_same_workspace_domain_has_only_one_manager(manager):
    with pytest.raises(ValueError, match='Another managed'):
        ProcessManager(manager.workspace, manager.state, manager.backend)


def test_preview_forbids_process_manager(manager):
    manager.state.preview = True
    with pytest.raises(ValueError, match='Preview'):
        ProcessManager(manager.workspace, manager.state, manager.backend)


def test_ownership_backup_and_stop_timeout(manager, monkeypatch):
    child = Mock(pid=123456, returncode=None)
    child.poll.return_value = None
    popen = Mock(return_value=child)
    monkeypatch.setattr('web_console_processes.subprocess.Popen', popen)
    monkeypatch.setattr('web_console_processes.threading.Thread.start', lambda self: None)
    killed = []
    monkeypatch.setattr('web_console_processes.os.killpg', lambda pid, sig: killed.append((pid, sig)))
    manager.start(dict(kind='calibration', side='right', stage='manual'))
    manager.backend.check_idle_graph.assert_called_once()
    assert popen.call_args.kwargs.get('shell', False) is False
    assert popen.call_args.kwargs['start_new_session'] is True
    assert len(list((manager.workspace / 'log/web_calibration_backups').glob('*'))) == 1
    for data in [dict(kind='control'), dict(kind='merge'), dict(kind='calibration', side='left', stage='direction')]:
        with pytest.raises(ValueError):
            manager.start(data)
    with pytest.raises(ValueError, match='external'):
        manager.stop('control')
    child.wait.side_effect = subprocess.TimeoutExpired('fake', 15)
    with pytest.raises(ValueError, match='no force kill'):
        manager.stop('calibration')
    assert killed == [(123456, signal.SIGINT)]
    assert manager.active('calibration')


def test_graph_conflict_does_not_launch_anything(manager, monkeypatch):
    popen = Mock()
    monkeypatch.setattr('web_console_processes.subprocess.Popen', popen)
    manager.backend.check_idle_graph.side_effect = ValueError('Other controller')
    with pytest.raises(ValueError, match='Other controller'):
        manager.start(dict(kind='control'))
    popen.assert_not_called()


def test_preview_recording_is_not_replayable(manager):
    directory = manager.workspace / 'recordings'
    directory.mkdir()
    (directory / 'web_preview_test.yaml').write_text('preview_only: true\n')
    with pytest.raises(ValueError, match='Preview recordings'):
        manager.plan(dict(kind='replay', file='web_preview_test.yaml'))
