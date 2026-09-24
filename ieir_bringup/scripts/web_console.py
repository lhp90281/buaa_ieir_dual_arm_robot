#!/usr/bin/env python3
"""Loopback-only web console. Attach to ROS, or run an isolated visual preview."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
from pathlib import Path
import secrets
import shlex
import signal
import threading
import time
from urllib.parse import unquote, urlsplit

import yaml

from web_console_core import ConsoleState, PreviewBackend, calibration_files, robot_model


class ConsoleApp:
    def __init__(self, workspace, web_root, preview=False, allow_control=False, gripper=False, manage_processes=False):
        self.workspace, self.web_root = Path(workspace).resolve(), Path(web_root).resolve()
        model, self.assets = robot_model(self.workspace / 'src/description', gripper)
        # Preserve CAD surfaces, including thin walls and holes; no display decimation.
        self.state = ConsoleState(model, preview)
        self.token, self.allow_control = secrets.token_urlsafe(32), allow_control or preview
        self.command_lock = threading.Lock()
        self.record_lock = threading.Lock()
        self.recording, self.last_recording = None, None
        self.closing = threading.Event()
        self.processes = None
        if preview:
            self.backend = PreviewBackend(self.state)
        else:
            from web_console_ros import RosBackend
            self.backend = RosBackend(self.state)
        if manage_processes and not preview:
            from web_console_processes import ProcessManager
            try:
                self.processes = ProcessManager(self.workspace, self.state, self.backend, gripper)
            except Exception:
                self.backend.close()
                raise
        self.recorder = threading.Thread(target=self.record_tick, daemon=True)
        self.recorder.start()

    def bootstrap(self):
        return dict(token=self.token, allow_control=self.allow_control, preview=self.state.preview,
                    manage_processes=self.processes is not None,
                    model=self.state.model, workspace=str(self.workspace))

    def snapshot(self):
        data = self.state.snapshot()
        data['processes'] = self.processes.snapshot() if self.processes else {}
        now = time.monotonic()
        for motor in data['motors']:
            motor['age'] = now - motor.pop('at', now)
            for key in ('position', 'velocity', 'torque'):
                if not math.isfinite(motor[key]):
                    motor[key] = None
        for pose in data['poses'].values():
            pose['age'] = now - pose.pop('at', now)
        for session in data['calibration'].values():
            session['age'] = now - session.pop('at', now)
        with self.record_lock:
            data['recording'] = (dict(name=self.recording['name'], count=len(self.recording['samples']),
                                      duration=now-self.recording['start']) if self.recording else None)
            data['last_recording'] = self.last_recording
        return data

    def configuration(self):
        directory = self.workspace / 'src/ros2_ws_config'
        commands = {}
        for side in ('left', 'right'):
            quote = lambda name: shlex.quote(str(directory / name))
            commands[side] = {
                'direction': 'ros2 run ieir_controllers joint_zero_calibration --mode direction '
                             f'--calibration-yaml {quote(f"joint_calibration_dual_{side}.yaml")} '
                             f'--output {quote(f"joint_directions_{side}.yaml")}',
                'limits': 'ros2 run ieir_controllers joint_zero_calibration --mode limit-preview '
                          f'--calibration-yaml {quote(f"joint_calibration_dual_{side}.yaml")} '
                          f'--output {quote(f"joint_calibration_dual_{side}_reviewed.yaml")}',
                'manual': 'ros2 run ieir_controllers joint_manual_calibration '
                          f'--calibration-yaml {quote(f"joint_calibration_dual_{side}_reviewed.yaml")} '
                          f'--directions-yaml {quote(f"joint_directions_{side}.yaml")} '
                          f'--output {quote(f"joint_offsets_{side}.yaml")}',
                'merge': f'python3 {quote("merge_offsets.py")}',
            }
        recordings = sorted((self.workspace / 'recordings').glob('web_*.yaml'), reverse=True)[:30]
        return dict(files=calibration_files(self.workspace), commands=commands,
                    recordings=[dict(name=p.name, bytes=p.stat().st_size,
                                      replay=None if p.name.startswith('web_preview_') else
                                      'ros2 launch ieir_bringup replay.launch.py '
                                      f'input:={shlex.quote(str(p))} time_scale:=0.5 ramp_in:=5.0')
                                for p in recordings if p.is_file() and not p.is_symlink()])

    def action(self, data):
        if self.closing.is_set():
            raise ValueError('Console is shutting down')
        if not isinstance(data, dict) or data.get('confirmed') is not True:
            raise ValueError('Explicit operator confirmation is required')
        action = data.get('action')
        if action == 'heartbeat':
            if not self.state.preview:
                self.backend.browser_seen_at = time.monotonic()
            return 'OK'
        if action in ('record_start', 'record_stop'):
            return self.record_action(action)
        if not self.allow_control:
            raise PermissionError('Read-only server; restart with --allow-control for ROS commands')
        if not self.command_lock.acquire(blocking=False):
            raise ValueError('An operation is already pending; inspect its result before another command')
        try:
            if self.closing.is_set():
                raise ValueError('Console is shutting down')
            if action in ('process_start', 'process_stop'):
                if not self.processes:
                    raise PermissionError('Node management is disabled (or preview mode)')
                result = (self.processes.start(data) if action == 'process_start'
                          else self.processes.stop(data.get('kind')))
            else:
                if (self.processes and any(self.processes.active(k) for k in ('calibration', 'merge'))
                        and action not in ('calibration_key', 'calibration_angle')):
                    raise ValueError('Calibration owns this session; stop it before controlling arms')
                result = self.backend.execute(action, data)
            self.state.log(f'{action}: {result}')
            return result
        except Exception as exc:
            self.state.log(f'{action}: {exc}', 'error')
            raise
        finally:
            self.command_lock.release()

    def record_action(self, action):
        with self.record_lock:
            if action == 'record_start':
                if self.recording:
                    raise ValueError('Recording already running')
                positions = self.state.fresh_joints()
                prefix = 'web_preview_' if self.state.preview else 'web_'
                self.recording = dict(name=f'{prefix}{time.strftime("%Y%m%d_%H%M%S")}_{secrets.token_hex(2)}.yaml',
                                      start=time.monotonic(), names=sorted(positions), samples=[])
                self.state.log('Recording started (read-only joint sampling, 20 Hz)')
                return 'Recording started; controller mode was not changed'
            return self.finish_recording()

    def finish_recording(self):
        record, self.recording = self.recording, None
        if not record:
            raise ValueError('No recording is active')
        if len(record['samples']) < 2:
            return 'Recording discarded: fewer than two valid samples'
        folder = self.workspace / 'recordings'
        folder.mkdir(exist_ok=True)
        path = folder / record['name']
        content = dict(joint_names=record['names'], rate_hz=20.0,
                       duration_sec=record['samples'][-1]['t'], num_samples=len(record['samples']),
                       preview_only=self.state.preview, samples=record['samples'])
        with path.open('x') as stream:
            yaml.safe_dump(content, stream, sort_keys=False)
        self.last_recording = record['name']
        self.state.log(f'Recording saved: {path}')
        return f'Recording saved: {record["name"]}'

    def record_tick(self):
        while not self.closing.wait(0.05):
            with self.record_lock:
                if not self.recording:
                    continue
                try:
                    record = self.recording
                    positions = self.state.fresh_joints(record['names'])
                    elapsed = time.monotonic() - record['start']
                    record['samples'].append(dict(t=elapsed, q=[positions[n] for n in record['names']]))
                    if elapsed >= 120:
                        self.finish_recording()
                except (ValueError, OSError) as exc:
                    self.state.log(f'Recording stopped: {exc}', 'warn')
                    try:
                        self.finish_recording()
                    except (ValueError, OSError):
                        pass

    def close(self):
        self.closing.set()
        with self.command_lock:
            if self.processes:
                self.processes.close()
        self.recorder.join(timeout=1)
        with self.record_lock:
            if self.recording:
                self.finish_recording()
        self.backend.close()


def handler_for(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def trusted(self, write=False):
            port = self.server.server_address[1]
            hosts = {f'127.0.0.1:{port}', f'localhost:{port}'}
            host = self.headers.get('Host')
            if host not in hosts or self.headers.get('Sec-Fetch-Site') == 'cross-site':
                raise PermissionError('Only same-origin loopback access is allowed')
            if write:
                if self.headers.get('Origin') != f'http://{host}':
                    raise PermissionError('Invalid Origin')
                if not secrets.compare_digest(self.headers.get('X-Console-Token', ''), app.token):
                    raise PermissionError('Invalid console token')
                if self.headers.get_content_type() != 'application/json':
                    raise ValueError('Use application/json')

        def send(self, data, content_type='application/json', status=200):
            if content_type == 'application/json':
                data = json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'unsafe-inline'; "
                             "style-src 'self' 'unsafe-inline'; connect-src 'self'; img-src 'self' data:; "
                             "frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def pose_stream(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.connection.settimeout(2)
            try:
                while not app.closing.is_set():
                    packet = json.dumps(app.state.pose_sample(), allow_nan=False, separators=(',', ':'))
                    self.wfile.write(f'data: {packet}\n\n'.encode())
                    self.wfile.flush()
                    if app.closing.wait(1 / 30):
                        break
            except (OSError, TimeoutError):
                pass  # A closed/slow browser never blocks ROS or another client.

        def do_GET(self):
            try:
                self.trusted()
                path = unquote(urlsplit(self.path).path)
                if path == '/api/bootstrap':
                    return self.send(app.bootstrap())
                if path == '/api/state':
                    return self.send(app.snapshot())
                if path == '/api/pose-stream':
                    return self.pose_stream()
                if path == '/api/config':
                    return self.send(app.configuration())
                if path.startswith('/mesh/'):
                    file = app.assets.get(path[len('/mesh/'):])
                    if file is None:
                        raise FileNotFoundError()
                else:
                    file = (app.web_root / ('index.html' if path == '/' else path.lstrip('/'))).resolve()
                    if not file.is_relative_to(app.web_root):
                        raise PermissionError('Path outside web root')
                self.send(file.read_bytes(), mimetypes.guess_type(str(file))[0] or 'application/octet-stream')
            except PermissionError as exc:
                self.send(dict(error=str(exc)), status=403)
            except (FileNotFoundError, IsADirectoryError):
                self.send(dict(error='Not found'), status=404)
            except Exception as exc:
                self.send(dict(error=str(exc)), status=500)

        def do_POST(self):
            try:
                self.trusted(write=True)
                if self.path != '/api/action':
                    return self.send(dict(error='Not found'), status=404)
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 16384:
                    raise ValueError('Invalid request size')
                self.connection.settimeout(3)
                data = json.loads(self.rfile.read(size))
                self.send(dict(message=app.action(data)))
            except PermissionError as exc:
                self.send(dict(error=str(exc)), status=403)
            except (ValueError, TypeError, KeyError) as exc:
                self.send(dict(error=str(exc)), status=400)
            except Exception as exc:
                app.state.log(str(exc), 'error')
                self.send(dict(error='Operation failed; inspect console log'), status=500)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--allow-control', action='store_true')
    parser.add_argument('--gripper', action='store_true')
    parser.add_argument('--manage-processes', action='store_true')
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error('port must be 1024..65535')
    source_web = Path(__file__).resolve().parents[1] / 'web'
    if not source_web.is_dir():
        from ament_index_python.packages import get_package_share_directory
        source_web = Path(get_package_share_directory('ieir_bringup')) / 'web'
    app = ConsoleApp(args.workspace, source_web, args.preview, args.allow_control, args.gripper, args.manage_processes)
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_for(app))
    except OSError:
        app.close()
        raise
    server.daemon_threads = True
    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print(f'IEIR console: http://127.0.0.1:{args.port} '
          f'({"PREVIEW / no ROS" if args.preview else "managed ROS" if app.processes else "existing ROS only"}; '
          f'{"control available" if app.allow_control else "read-only"})', flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        app.close()


if __name__ == '__main__':
    main()
