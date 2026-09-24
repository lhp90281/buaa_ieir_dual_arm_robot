"""Local-console models and validation. No ROS import or motor side effects."""
from collections import deque
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import threading
import time
import xml.etree.ElementTree as ET

import yaml


GRAVITY = 'gravity_compensation_controller'
JOINT = 'joint_position_controller'
CARTESIAN = 'cartesian_position_controller'
MODES = {'gravity': GRAVITY, 'joint': JOINT, 'cartesian': CARTESIAN}


def number(value, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('Expected a numeric value')
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'Value must be finite and within [{low}, {high}]')
    return value


def vector(text, default='0 0 0'):
    return [float(x) for x in (text or default).split()]


def robot_model(description_root, gripper=False):
    """Expose only mesh paths explicitly referenced by the selected URDF."""
    root = Path(description_root)
    robot = ET.parse(root / 'dual_arm_support/urdf/dual_arm_robot_plug.urdf').getroot()
    assets, links, joints = {}, [], []
    for link in robot.findall('link'):
        name = link.get('name')
        if not gripper and 'gripper' in name:
            continue
        visuals = []
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            uri = mesh.get('filename')
            if not uri.startswith('package://'):
                raise ValueError(f'Unsupported mesh: {uri}')
            path = (root / uri[len('package://'):]).resolve()
            if not path.is_relative_to(root.resolve()) or not path.is_file():
                raise ValueError(f'Missing mesh: {uri}')
            key = str(len(assets))
            assets[key] = path
            origin = visual.find('origin')
            visuals.append(dict(mesh=f'/mesh/{key}', xyz=vector(origin.get('xyz') if origin is not None else None),
                                rpy=vector(origin.get('rpy') if origin is not None else None),
                                scale=vector(mesh.get('scale'), '1 1 1')))
        links.append(dict(name=name, visuals=visuals))
    names = {link['name'] for link in links}
    for joint in robot.findall('joint'):
        parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
        if child not in names or parent not in names:
            continue
        origin, axis, limit = joint.find('origin'), joint.find('axis'), joint.find('limit')
        joints.append(dict(name=joint.get('name'), type=joint.get('type'), parent=parent, child=child,
                           xyz=vector(origin.get('xyz') if origin is not None else None),
                           rpy=vector(origin.get('rpy') if origin is not None else None),
                           axis=vector(axis.get('xyz') if axis is not None else None, '1 0 0'),
                           lower=float(limit.get('lower', '-3.14159')) if limit is not None else 0,
                           upper=float(limit.get('upper', '3.14159')) if limit is not None else 0))
    return dict(links=links, joints=joints, frame='base_footprint', gripper=gripper), assets


def display_assets(assets, directory):
    """Use bundled display-only LOD only when the source mesh fingerprint matches."""
    directory = Path(directory).resolve()
    try:
        manifest = json.loads((directory / 'manifest.json').read_text())
    except (OSError, ValueError):
        return assets
    resolved = {}
    for key, source in assets.items():
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        candidate = (directory / manifest.get(digest, {}).get('file', '')).resolve()
        resolved[key] = candidate if candidate.is_relative_to(directory) and candidate.is_file() else source
    return resolved


class ConsoleState:
    def __init__(self, model, preview=False):
        self.lock = threading.RLock()
        self.model, self.preview = model, preview
        self.limits = {j['name']: (j['lower'], j['upper']) for j in model['joints']
                       if re.fullmatch(r'(left|right)_joint_[0-6]', j['name'])}
        self.joints, self.motors, self.controllers, self.poses = {}, [], {}, {}
        self.controller_at = 0.0
        self.services, self.nodes, self.calibration = [], [], {}
        self.logs = deque(maxlen=160)
        self.sequence = 0

    def log(self, text, level='info', source='console'):
        with self.lock:
            self.sequence += 1
            self.logs.append(dict(id=self.sequence, time=time.strftime('%H:%M:%S'),
                                  level=level, source=source, text=str(text)[:1200]))

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            joints = {name: {**data, 'age': now - data['at']} for name, data in self.joints.items()}
            return dict(preview=self.preview, joints=joints, motors=copy.deepcopy(self.motors),
                        controllers=dict(self.controllers), controller_age=now - self.controller_at,
                        poses=copy.deepcopy(self.poses), services=list(self.services), nodes=list(self.nodes),
                        calibration=copy.deepcopy(self.calibration), logs=list(self.logs))

    def fresh_joints(self, names=None):
        with self.lock:
            names = list(names if names is not None else self.joints)
            if not names or any(n not in self.limits for n in names):
                raise ValueError('No known arm joints selected')
            now = time.monotonic()
            for name in names:
                value = self.joints.get(name)
                if value is None or now - value['at'] > 0.5 or not math.isfinite(value['position']):
                    raise ValueError(f'Stale or missing joint feedback: {name}')
            return {n: self.joints[n]['position'] for n in names}

    def pose_sample(self):
        """Small read-only packet; stale joints are not invented or extrapolated."""
        with self.lock:
            now = time.monotonic()
            return dict(positions={n: j['position'] for n, j in self.joints.items()
                                   if now - j['at'] <= 0.5 and math.isfinite(j['position'])})

    def zero_plan(self, data):
        side = data.get('side')
        if side not in ('left', 'right', 'dual'):
            raise ValueError('Select left, right or dual for model-zero return')
        sides = ('left', 'right') if side == 'dual' else (side,)
        current = self.fresh_joints([f'{s}_joint_{i}' for s in sides for i in range(7)])
        duration = number(data.get('duration'), 3, 60)
        for name in current:
            number(0.0, *self.limits[name])
        # Quintic easing peaks at 1.875 * distance / duration; cap at 20 deg/s.
        duration = max(duration, 1.875 * max(abs(q) for q in current.values()) / math.radians(20))
        duration = math.ceil(duration * 100) / 100
        if duration > 60:
            raise ValueError('Return duration exceeds 60 seconds; check calibration/feedback')
        return current, duration

    def require_controller(self, name):
        with self.lock:
            if time.monotonic() - self.controller_at > 3 or self.controllers.get(name) != 'active':
                raise ValueError(f'Controller is not confirmed active: {name}')

    def joint_target(self, data):
        targets = data.get('positions')
        if not isinstance(targets, dict) or not targets:
            raise ValueError('positions must contain named joint targets')
        duration = number(data.get('duration'), 1, 60)
        current = self.fresh_joints(targets)
        self.require_controller(JOINT)
        self.require_controller(GRAVITY)
        for name, target in targets.items():
            number(target, *self.limits[name])
            if abs(target - current[name]) > math.radians(20) + 1e-8:
                raise ValueError(f'{name}: one web command is limited to 20 degrees from feedback')
        return targets, duration

    def cartesian_target(self, data):
        side = data.get('side')
        if side not in ('left', 'right'):
            raise ValueError('Unknown arm')
        self.fresh_joints([f'{side}_joint_{i}' for i in range(7)])
        self.require_controller(CARTESIAN)
        self.require_controller(GRAVITY)
        pose = self.poses.get(side)
        if not pose or time.monotonic() - pose['at'] > 0.5:
            raise ValueError('Fresh base_footprint end-effector TF is required')
        p, q = data.get('position'), data.get('quaternion')
        if not isinstance(p, list) or len(p) != 3 or not isinstance(q, list) or len(q) != 4:
            raise ValueError('Expected position[3], quaternion[4]')
        p = [number(v, -5, 5) for v in p]
        q = [number(v, -1, 1) for v in q]
        norm = math.sqrt(sum(v*v for v in q))
        if abs(norm - 1) > 0.01:
            raise ValueError('Quaternion must be normalized')
        q = [v/norm for v in q]
        if math.dist(p, pose['position']) > 0.05 + 1e-8:
            raise ValueError('One web target is limited to 50 mm from current TF')
        angle = 2 * math.acos(min(1, abs(sum(a*b for a, b in zip(q, pose['quaternion'])))))
        if angle > math.radians(10) + 1e-8:
            raise ValueError('One web target is limited to 10 degrees from current TF')
        return side, p, q


def switch_plan(controllers, mode):
    if mode not in MODES:
        raise ValueError('Unknown controller mode')
    target = MODES[mode]
    required = {GRAVITY, target}
    for name in required:
        if controllers.get(name) not in ('active', 'inactive'):
            raise ValueError(f'Controller is not loaded/configured: {name}')
    activate = sorted(n for n in required if controllers[n] != 'active')
    deactivate = [n for n in (JOINT, CARTESIAN) if n != target and controllers.get(n) == 'active']
    return activate, deactivate


def zero_trajectory(current, duration):
    """Sample a zero-end-velocity ramp for the existing linear-segment controller."""
    count = math.ceil(duration * 50)
    for i in range(count + 1):
        u = i / count
        blend = u*u*u * (10 + u * (-15 + 6*u))
        yield duration * u, [q * (1 - blend) for q in current.values()]


def calibration_files(workspace):
    directory = Path(workspace) / 'src/ros2_ws_config'
    names = ['friction_model.yaml', 'joint_offsets_dual.yaml']
    for side in ('left', 'right'):
        names += [f'joint_directions_{side}.yaml', f'joint_calibration_dual_{side}_reviewed.yaml',
                  f'joint_offsets_{side}.yaml']
    output = []
    for name in names:
        path = directory / name
        entry = dict(name=name, exists=path.is_file(), entries=0, status='missing')
        if path.is_file():
            try:
                data = yaml.safe_load(path.read_text())
                if not isinstance(data, dict):
                    raise ValueError('Expected YAML mapping')
                entry.update(status='readable', entries=len(data.get('offsets', data.get('directions', data.get('joints', [])))))
            except (ValueError, OSError, yaml.YAMLError) as exc:
                entry.update(status='invalid', error=str(exc))
        output.append(entry)
    return output


class PreviewBackend:
    """Explicitly synthetic, never imports ROS or opens a CAN device."""
    def __init__(self, state):
        self.state = state
        self.targets = {n: 0.0 for n in state.limits}
        self.running = True
        state.controllers = {GRAVITY: 'active', JOINT: 'inactive', CARTESIAN: 'inactive'}
        state.services = [f'/teleop/{x}' for x in ('prepare', 'enable', 'disable', 'exit')]
        state.nodes = ['preview_only_no_ros']
        state.log('Preview backend: synthetic data, no ROS connection')
        self.thread = threading.Thread(target=self.tick, daemon=True)
        self.thread.start()

    def tick(self):
        while self.running:
            with self.state.lock:
                now = time.monotonic()
                self.state.controller_at = now
                for name, target in self.targets.items():
                    current = self.state.joints.get(name, {}).get('position', 0.0)
                    position = current + (target-current) * 0.12
                    self.state.joints[name] = dict(position=position, velocity=(position-current)*20,
                                                   effort=0.0, at=now)
                self.state.motors = [dict(channel=c, slot=i, online=True, enabled=True,
                                          position=self.state.joints[f'{s}_joint_{i}']['position'],
                                          velocity=0.0, torque=0.0, error=1, age=0)
                                     for c, s in enumerate(('left', 'right')) for i in range(7)]
            time.sleep(0.05)

    def execute(self, action, data):
        if action == 'switch':
            add, remove = switch_plan(self.state.controllers, data.get('mode'))
            with self.state.lock:
                for name in add:
                    self.state.controllers[name] = 'active'
                for name in remove:
                    self.state.controllers[name] = 'inactive'
        elif action == 'joint':
            targets, _ = self.state.joint_target(data)
            self.targets.update(targets)
        elif action == 'zero':
            current, _ = self.state.zero_plan(data)
            self.execute('switch', dict(mode='joint'))
            self.targets.update({n: 0.0 for n in current})
        elif action == 'teleop':
            if data.get('operation') not in ('prepare', 'enable', 'disable', 'exit'):
                raise ValueError('Unknown teleop operation')
        else:
            raise ValueError('This action requires live ROS; preview does not fabricate execution')
        return 'Preview action completed; no robot command sent'

    def close(self):
        self.running = False
        self.thread.join(timeout=1)
