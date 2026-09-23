"""Motor-delta direction check. No absolute position or motor commands in the viewer."""
import math
import os
import queue
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState
from std_msgs.msg import String


class DirectionCheck:
    def __init__(self, joints, min_motion=0.08):
        self.joints = joints
        self.index = 0
        self.signs = [1] * len(joints)
        self.confirmed = [False] * len(joints)
        self.min_motion = min_motion
        self.reset()

    @property
    def done(self):
        return self.index == len(self.joints)

    def reset(self):
        self.baseline = None
        self.last_raw = None
        self.delta = 0.0
        self.motion = 0.0
        self.blocked = False
        self.notice = 'Place near zero. ENTER captures a display-only origin.'

    def update(self, raw):
        if self.done or self.baseline is None or self.blocked:
            return
        if raw is None or not math.isfinite(raw):
            self.blocked = True
            self.notice = 'Feedback lost. Restore feedback, then R to reset.'
            return
        if abs(raw - self.last_raw) > 0.35 or abs(raw - self.baseline) > 1.0:
            self.blocked = True
            self.notice = 'Large delta / encoder wrap. Return near zero; R to reset.'
            return
        self.last_raw = raw
        self.delta = raw - self.baseline
        self.motion = max(self.motion, abs(self.delta))

    def key(self, key, raw):
        if self.done:
            return
        if key == 'r':
            self.reset()
        elif key == 'backspace' and self.index > 0:
            self.index -= 1
            self.confirmed[self.index:] = [False] * (len(self.joints) - self.index)
            self.reset()
        elif key == 'space':
            self.signs[self.index] *= -1
        elif key == 'enter':
            if raw is None or not math.isfinite(raw) or self.blocked:
                self.notice = 'No valid feedback. Restore feedback, then R.'
            elif self.baseline is None:
                self.baseline = self.last_raw = raw
                self.notice = 'Move this joint gently. SPACE flips; ENTER accepts.'
            elif self.motion < self.min_motion:
                self.notice = 'Move at least %.1f deg before accepting.' % math.degrees(self.min_motion)
            else:
                self.confirmed[self.index] = True
                self.index += 1
                self.reset()

    def preview(self):
        if self.done:
            return '', 0.0
        return self.joints[self.index].name, self.signs[self.index] * self.delta

    def results(self):
        if not self.done or not all(self.confirmed):
            raise ValueError('All joint directions must be confirmed before saving')
        return [dict(name=j.name, slot=j.slot, axis_sign=s, direction_verified=True)
                for j, s in zip(self.joints, self.signs)]


def load_directions(data, cfg):
    if data.get('mode') != 'direction' or data.get('channel') != cfg.channel:
        raise ValueError('Direction file mode/channel does not match this arm')
    entries = data.get('directions', [])
    by_name = {e['name']: e for e in entries}
    if len(by_name) != len(entries) or set(by_name) != {j.name for j in cfg.joints}:
        raise ValueError('Direction file must contain each configured joint exactly once')
    for joint in cfg.joints:
        entry = by_name[joint.name]
        if (entry.get('slot') != joint.slot or entry.get('axis_sign') not in (-1, 1)
                or entry.get('direction_verified') is not True):
            raise ValueError(f'Invalid/unconfirmed direction for {joint.name}')
        joint.axis_sign = int(entry['axis_sign'])
        joint.direction_verified = True


class LimitReview:
    """Stage editable reference angles; only apply after every joint is confirmed."""
    ORDER = (6, 5, 4, 2, 3, 1, 0)

    def __init__(self, joints, bounds):
        by_slot = {joint.slot: joint for joint in joints}
        if not joints or len(by_slot) != len(joints) or not set(by_slot).issubset(self.ORDER):
            raise ValueError('Limit review supports unique arm slots 0..6')
        self.joints = [by_slot[slot] for slot in self.ORDER if slot in by_slot]
        self.bounds = bounds
        self.angles = [j.urdf_pos_at_limit for j in self.joints]
        if any(not math.isfinite(q) or abs(q) > 2 * math.pi for q in self.angles):
            raise ValueError('Reference angles must be finite and within +/-360 deg')
        self.index = 0
        self.confirmed = [False] * len(joints)
        self.valid_edit = True
        self.notice = 'Check the reference pose; ENTER confirms this joint.'

    @property
    def done(self):
        return self.index == len(self.joints)

    def key(self, key):
        if self.done:
            return
        q = self.angles[self.index]
        changed = False
        if key == 'invalid_angle':
            self.valid_edit = False
            self.notice = 'Invalid angle. Edit again or R to restore before confirming.'
            return
        if key.startswith('angle_deg:'):
            try:
                q = math.radians(float(key.split(':', 1)[1]))
            except ValueError:
                self.valid_edit = False
                self.notice = 'Invalid angle. Enter a numeric value in degrees.'
                return
            changed = True
        elif key in ('left', 'right', 'left_fine', 'right_fine'):
            q += math.radians((0.1 if key.endswith('_fine') else 1.0) *
                              (-1 if key.startswith('left') else 1))
            changed = True
        elif key == 'r':
            q = self.joints[self.index].urdf_pos_at_limit
            changed = True
        elif key == 'backspace' and self.index:
            self.index -= 1
            self.confirmed[self.index:] = [False] * (len(self.joints) - self.index)
            self.valid_edit = True
        elif key == 'enter':
            if not self.valid_edit:
                return
            self.confirmed[self.index] = True
            self.index += 1
            self.notice = 'Check the reference pose; ENTER confirms this joint.'
        if changed:
            if not math.isfinite(q) or abs(q) > 2 * math.pi:
                self.valid_edit = False
                self.notice = 'Angle rejected: use a finite value within +/-360 deg.'
                return
            self.angles[self.index] = q
            self.valid_edit = True
            self.confirmed[self.index] = False
            self.notice = 'Edited reference. Check the model, then ENTER to confirm.'

    def apply(self):
        if not self.done or not all(self.confirmed):
            raise ValueError('All displayed limits must be confirmed')
        for joint, q in zip(self.joints, self.angles):
            joint.reference_original = joint.urdf_pos_at_limit
            joint.urdf_pos_at_limit = q
            joint.reference_confirmed = True


def model_joint_bounds(joints):
    path = Path(get_package_share_directory('dual_arm_support')) / 'urdf/dual_arm_robot_plug.urdf'
    root = ET.parse(path).getroot()
    bounds = {}
    for joint in joints:
        limit = root.find(f"joint[@name='{joint.name}']/limit")
        if limit is None:
            raise ValueError(f'No URDF joint limits for {joint.name}')
        bounds[joint.name] = (float(limit.get('lower')), float(limit.get('upper')))
    return bounds


class DirectionPreview:
    """Dedicated visualization process; only keyboard events come back from it."""
    def __init__(self, node, cfg, limit_review=False, motion_workflow=False):
        self.node, self.cfg = node, cfg
        self.prefix = '/calibration_preview_' + uuid.uuid4().hex
        self.keys = queue.Queue()
        self.pose_pub = node.create_publisher(JointState, self.prefix + '/pose', 10)
        self.text_pub = node.create_publisher(String, self.prefix + '/status', 10)
        self.key_sub = node.create_subscription(
            String, self.prefix + '/key', lambda msg: self.keys.put(msg.data), 10)
        self.process = subprocess.Popen([
            'ros2', 'run', 'eiriarm_mujoco', 'calibration_viewer', '--ros-args',
            '-p', 'topic_prefix:=' + self.prefix,
            '-p', 'motion_workflow:=' + str(motion_workflow).lower(),
            '-p', 'limit_edit:=' + str(limit_review).lower()], start_new_session=True)

    def wait_ready(self, stop_evt):
        deadline = time.monotonic() + 30.0
        while not stop_evt.is_set() and time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError('MuJoCo preview failed to open; no motors were enabled')
            if self.pose_pub.get_subscription_count() and self.text_pub.get_subscription_count():
                return
            time.sleep(0.05)
        raise RuntimeError('Preview startup cancelled/timed out; no motors were enabled')

    def run(self, stop_evt):
        check = DirectionCheck(self.cfg.joints)
        # Keys pressed while waiting for motor enable must not confirm anything.
        while not self.keys.empty():
            self.keys.get_nowait()
        while not stop_evt.is_set() and not check.done:
            if self.process.poll() is not None:
                stop_evt.set()
                break
            joint = self.cfg.joints[check.index]
            state = self.node.get_state(joint.slot)
            raw = state[0] if state else None
            check.update(raw)
            try:
                key = self.keys.get_nowait()
                if key == 'escape':
                    stop_evt.set()
                    break
                check.key(key, raw)
            except queue.Empty:
                pass
            name, angle = check.preview()
            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            if name:
                msg.name, msg.position = [name], [angle]
            self.pose_pub.publish(msg)
            sign = check.signs[check.index] if not check.done else 0
            self.text_pub.publish(String(data=(
                f'DIRECTION CHECK  |  can{self.cfg.channel}  |  '
                f'{min(check.index + 1, len(check.joints))}/{len(check.joints)}\n'
                f'{name}   sign={sign:+d}   delta={math.degrees(angle):+.1f} deg\n'
                f'{check.notice}\n'
                'SPACE: flip  ENTER: capture/next  R: reset  BACKSPACE: previous  ESC: abort\n'
                'Display only. Other joints stay at zero. Support the real arm.')))
            time.sleep(0.02)
        return None if stop_evt.is_set() else check.results()

    def review_limits(self, stop_evt):
        review = LimitReview(self.cfg.joints, model_joint_bounds(self.cfg.joints))
        while not stop_evt.is_set() and not review.done:
            if self.process.poll() is not None:
                stop_evt.set()
                break
            rclpy.spin_once(self.node, timeout_sec=0.01)
            try:
                key = self.keys.get_nowait()
                if key == 'escape':
                    stop_evt.set()
                    break
                review.key(key)
            except queue.Empty:
                pass
            if review.done:
                break
            joint = review.joints[review.index]
            angle = review.angles[review.index]
            low, high = review.bounds[joint.name]
            outside = angle < low - 1e-4 or angle > high + 1e-4
            msg = JointState(name=[joint.name], position=[angle])
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.pose_pub.publish(msg)
            self.text_pub.publish(String(data=(
                f'LIMIT REFERENCE REVIEW  |  can{self.cfg.channel}  |  '
                f'Joint {joint.slot + 1} ({joint.name})\n'
                f'Side: {joint.limit_side.upper()}   Reference: {math.degrees(angle):+.2f} deg '
                f'({angle:+.5f} rad)\n'
                f'Original YAML: {math.degrees(joint.urdf_pos_at_limit):+.2f} deg   '
                f'URDF range: [{math.degrees(low):+.2f}, {math.degrees(high):+.2f}] deg\n'
                f'{review.notice}\n'
                'E: type degrees  LEFT/RIGHT: 1 deg  SHIFT: 0.1 deg  ENTER: confirm\n'
                'R: restore  BACKSPACE: previous  ESC: cancel\n'
                + ('OUTSIDE URDF RANGE: verify the mechanical stop angle.\n' if outside else '')
                + 'Model preview only. Other joints are at zero; this is not a collision check.')))
            time.sleep(0.02)
        if stop_evt.is_set():
            return False
        review.apply()
        return True

    def close(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
