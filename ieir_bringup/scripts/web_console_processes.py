"""Owned, allowlisted ROS processes. No shell, bridge startup, or arbitrary argv."""
import fcntl
import ipaddress
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time

import yaml


KINDS = ('control', 'calibration', 'merge', 'teleop', 'replay')


class ProcessManager:
    def __init__(self, workspace, state, backend, gripper=False):
        self.workspace, self.state, self.backend = Path(workspace).resolve(), state, backend
        self.directory = self.workspace / 'src/ros2_ws_config'
        self.gripper = str(gripper).lower()
        self.jobs, self.lock = {}, threading.RLock()
        if state.preview:
            raise ValueError('Preview must never launch ROS processes')
        # The lock is per workspace/domain, not HTTP port. Do not unlink a held lock.
        self.guard = (self.workspace / f'.web-console-{os.environ.get("ROS_DOMAIN_ID", "0")}.lock').open('a')
        try:
            fcntl.flock(self.guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.guard.close()
            raise ValueError('Another managed Web UI owns this workspace/domain') from None

    def active(self, kind):
        with self.lock:
            job = self.jobs.get(kind)
            return bool(job and job['process'].poll() is None)

    def snapshot(self):
        with self.lock:
            return {kind: dict(pid=j['process'].pid, running=j['process'].poll() is None,
                               exit_code=j['process'].poll(), label=j['label'], stopping=j['stopping'])
                    for kind, j in self.jobs.items()}

    def path(self, name, required=True):
        path = self.directory / name
        if path.is_symlink() or not path.resolve().is_relative_to(self.directory.resolve()):
            raise ValueError('Configuration must be a local regular file')
        if required and not path.is_file():
            raise ValueError(f'Missing configuration: {path}')
        return str(path)

    def plan(self, data):
        kind = data.get('kind')
        if kind not in KINDS:
            raise ValueError('Unknown process kind')
        outputs = []
        if kind == 'control':
            arms = data.get('arms', 'dual')
            if arms not in ('left', 'right', 'dual'):
                raise ValueError('arms must be left/right/dual')
            offsets = self.path(f'joint_offsets_{arms}.yaml')
            entries = yaml.safe_load(Path(offsets).read_text()).get('offsets', [])
            sides = ('left', 'right') if arms == 'dual' else (arms,)
            expected = {f'{side}_joint_{i}' for side in sides for i in range(7)}
            selected = [e for e in entries if e.get('name') in expected]
            if len(selected) != len(expected) or {e['name'] for e in selected} != expected:
                raise ValueError('Complete calibration is required before controller startup')
            for e in selected:
                channel = 0 if e['name'].startswith('left_') else 1
                if (float(e.get('axis_sign', 0)) not in (-1, 1)
                        or not math.isfinite(float(e['zero_offset']))
                        or int(e.get('channel', channel)) != channel
                        or int(e['slot']) != int(e['name'].rsplit('_', 1)[1])):
                    raise ValueError('Invalid calibration sign/offset/channel/slot')
            command = ['ros2', 'launch', 'ieir_bringup', 'real_robot.launch.py',
                       f'arms:={arms}', 'controller:=gravity', f'gripper:={self.gripper}',
                       'use_web:=false', 'use_gui:=false', 'teleop:=false',
                       f'offsets_yaml:={offsets}',
                       f'friction_model_yaml:={self.path("friction_model.yaml")}']
            label = f'control / {arms} / gravity'
        elif kind == 'calibration':
            side, stage = data.get('side'), data.get('stage')
            if side not in ('left', 'right') or stage not in ('direction', 'limits', 'manual'):
                raise ValueError('Unknown calibration side/stage')
            reference = self.path(f'joint_calibration_dual_{side}{"_reviewed" if stage == "manual" else ""}.yaml')
            config = yaml.safe_load(Path(reference).read_text())
            if config.get('channel') != (0 if side == 'left' else 1):
                raise ValueError('Calibration file channel disagrees with selected arm')
            command = ['ros2', 'run', 'ieir_controllers']
            if stage == 'manual':
                command += ['joint_manual_calibration', '--directions-yaml', self.path(f'joint_directions_{side}.yaml')]
                joint = data.get('joint')
                if joint is not None:
                    if type(joint) is not int or joint not in range(1, 8):
                        raise ValueError('joint must be 1..7 or null for all joints')
                    command += ['--joint', str(joint)]
                output = self.path(f'joint_offsets_{side}.yaml', False)
            else:
                command += ['joint_zero_calibration', '--mode', 'direction' if stage == 'direction' else 'limit-preview']
                output = self.path(f'joint_directions_{side}.yaml' if stage == 'direction'
                                   else f'joint_calibration_dual_{side}_reviewed.yaml', False)
            command += ['--calibration-yaml', reference, '--output', output, '--preview-backend', 'web']
            outputs.append(output)
            label = f'{side} / {stage}'
        elif kind == 'merge':
            output = self.path('joint_offsets_dual.yaml', False)
            command = ['python3', self.path('merge_offsets.py'), '--left', self.path('joint_offsets_left.yaml'),
                       '--right', self.path('joint_offsets_right.yaml'), '--output', output]
            outputs.append(output)
            label = 'merge / 14 joints'
        elif kind == 'teleop':
            role = data.get('role')
            if role not in ('master', 'slave'):
                raise ValueError('Unknown teleop role')
            host = str(ipaddress.IPv4Address(data.get('peer_host', '')))
            ports = [data.get(k) for k in ('local_port', 'peer_port')]
            if any(type(p) is not int or not 1024 <= p <= 65535 for p in ports):
                raise ValueError('UDP ports must be 1024..65535')
            command = ['ros2', 'launch', 'ieir_bringup', 'teleop.launch.py', f'role:={role}',
                       'mode:=no_feedback', f'gripper:={self.gripper}', f'peer_host:={host}',
                       f'local_port:={ports[0]}', f'peer_port:={ports[1]}']
            label = f'teleop / {role} / {host}'
        else:
            name = data.get('file', '')
            if not isinstance(name, str) or Path(name).name != name or not name.startswith('web_') or not name.endswith('.yaml'):
                raise ValueError('Select a recording from this console')
            path = self.workspace / 'recordings' / name
            if path.is_symlink() or path.stat().st_size > 8_000_000:
                raise ValueError('Invalid recording file')
            record = yaml.safe_load(path.read_text())
            if record.get('preview_only', True):
                raise ValueError('Preview recordings cannot be replayed on hardware')
            names, samples = record['joint_names'], record['samples']
            if not names or len(set(names)) != len(names) or not set(names) <= self.state.limits.keys() or len(samples) < 2:
                raise ValueError('Invalid recording joint names/samples')
            previous = -1.0
            for sample in samples:
                stamp, positions = float(sample['t']), sample['q']
                if not math.isfinite(stamp) or stamp <= previous or len(positions) != len(names):
                    raise ValueError('Invalid recording time/positions')
                previous = stamp
                for joint, q in zip(names, positions):
                    low, high = self.state.limits[joint]
                    if not math.isfinite(float(q)) or not low <= q <= high:
                        raise ValueError('Recording exceeds joint limits')
            command = ['ros2', 'launch', 'ieir_bringup', 'replay.launch.py', f'input:={path}',
                       'time_scale:=0.5', 'ramp_in:=5.0', 'publish_rate:=50']
            label = f'replay / {name}'
        return kind, command, outputs, label

    def start(self, data):
        kind, command, outputs, label = self.plan(data)
        with self.lock:
            if self.active(kind):
                raise ValueError('This process is already running; no duplicate start')
            if kind in ('control', 'calibration', 'merge'):
                if any(self.active(k) for k in KINDS):
                    raise ValueError('Stop existing managed processes first; control and calibration are mutually exclusive')
                self.backend.check_idle_graph()
            else:
                if any(self.active(k) for k in ('calibration', 'merge', 'teleop', 'replay')):
                    raise ValueError('Stop calibration/teleop/replay before another target producer')
                if kind == 'replay':
                    record = yaml.safe_load((self.workspace / 'recordings' / data['file']).read_text())
                    self.backend.check_motor_feedback(self.state.fresh_joints(record['joint_names']))
                self.backend.check_target_start(kind)
            for output in outputs:
                path = Path(output)
                if path.exists():
                    folder = self.workspace / 'log/web_calibration_backups'
                    folder.mkdir(parents=True, exist_ok=True)
                    backup = folder / f'{time.time_ns()}_{path.name}'
                    shutil.copy2(path, backup)
                    self.state.log(f'Calibration backup: {backup}')
            process = subprocess.Popen(command, cwd=self.workspace, stdin=subprocess.DEVNULL,
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                       env={**os.environ, 'PYTHONUNBUFFERED': '1'}, start_new_session=True)
            self.jobs[kind] = dict(process=process, label=label, stopping=False)
            threading.Thread(target=self.read_output, args=(kind, process), daemon=True).start()
        self.state.log(f'Started {label}; PID {process.pid}. Readiness still requires ROS feedback.')
        return f'Start requested: {label}; inspect process log and ROS state'

    def read_output(self, kind, process):
        try:
            for line in process.stdout:
                self.state.log(line.rstrip()[:2000], source=kind)
        finally:
            process.stdout.close()
            code = process.wait()
            self.state.log(f'{kind} exited: {code}', 'info' if code == 0 else 'warn')

    def stop(self, kind):
        if kind not in KINDS:
            raise ValueError('Unknown process kind')
        if kind == 'control' and any(self.active(k) for k in ('teleop', 'replay')):
            raise ValueError('Stop teleop/replay before stopping the controller stack')
        with self.lock:
            job = self.jobs.get(kind)
            if not job or job['process'].poll() is not None:
                raise ValueError('No owned running process; external processes are never terminated')
            process = job['process']
            job['stopping'] = True
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            raise ValueError('Graceful stop timed out; no force kill. Support robot/use physical stop and inspect logs.') from None
        return f'{kind} exited ({process.returncode}); confirm motor state before continuing'

    def close(self):
        for kind in ('replay', 'teleop', 'calibration', 'merge', 'control'):
            if self.active(kind):
                try:
                    self.stop(kind)
                except Exception as exc:
                    self.state.log(f'Shutdown: {exc}', 'error')
        self.guard.close()
