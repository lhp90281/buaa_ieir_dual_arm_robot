#!/usr/bin/env python3
"""Observe and record motor torque residuals during teleoperation.

This node does not command the robot. It subscribes to /joint_states, reads the
running gravity_compensation_controller parameters, recomputes the compensation
terms, then records and plots:

    tau_residual = tau_measured - tau_gravity_scaled - tau_friction

Use this first to judge whether DM torque feedback is stable enough for a
future force-feedback path.
"""

import argparse
import csv
import math
import signal
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState


PARAM_BOOL = 1
PARAM_DOUBLE = 3
PARAM_STRING = 4
PARAM_STRING_ARRAY = 9
PARAM_DOUBLE_ARRAY = 7


class ForceObserver(Node):
    def __init__(self, args):
        super().__init__('teleop_force_observer')
        self.args = args
        self.start_wall = time.time()
        self.start_mono = time.monotonic()
        self.lock = threading.Lock()

        ctrl = args.gravity_controller
        self.params = self._load_controller_params(ctrl)
        self.joint_names = list(self.params['joints'])
        if not self.joint_names:
            raise RuntimeError('gravity controller has empty joints parameter')

        self.model = self._load_pinocchio_model(args.robot_description_node)
        self.data = self.model.createData()
        self.model.gravity.linear = np.array([0.0, 0.0, -9.81])
        self.q = np.zeros(self.model.nq)
        self.v = np.zeros(self.model.nv)
        self.v_filtered = np.zeros(self.model.nv)
        self.joint_idx_v = self._build_joint_index_map()

        self.friction_models = self._load_friction_models()

        self.csv_file = Path(args.output).expanduser()
        self.csv_file.parent.mkdir(parents=True, exist_ok=True)
        self.csv_fh = self.csv_file.open('w', newline='')
        self.csv_writer = csv.writer(self.csv_fh)
        self._write_header()

        self.latest: Optional[Dict[str, np.ndarray]] = None
        self.history = {
            name: deque(maxlen=max(10, int(args.window * args.plot_rate)))
            for name in self.joint_names
        }
        self.last_record_mono = 0.0
        self.last_plot_mono = 0.0
        self.sample_count = 0

        self.sub = self.create_subscription(
            JointState,
            args.joint_state_topic,
            self._on_joint_state,
            qos_profile_sensor_data)

        self.get_logger().info(
            f'Force observer ready: joints={len(self.joint_names)} '
            f'output={self.csv_file}')
        if self.params['friction_enabled']:
            self.get_logger().info(
                f"friction model: {self.params['friction_model_yaml']} "
                f"deadband={self.params['friction_deadband']:.3f}")
        else:
            self.get_logger().warn('friction compensation is disabled in parameters')

    # ------------------------------------------------------------------
    # Parameter / model loading
    # ------------------------------------------------------------------
    def _call_get_parameters(self, node_name: str, names: List[str], timeout=5.0):
        client = self.create_client(GetParameters, f'/{node_name}/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'/{node_name}/get_parameters not available')
            req = GetParameters.Request()
            req.names = names
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            resp = future.result()
            if resp is None:
                raise RuntimeError(f'/{node_name}/get_parameters timed out')
            return resp.values
        finally:
            self.destroy_client(client)

    def _load_controller_params(self, controller_name: str) -> Dict[str, object]:
        names = [
            'joints',
            'gravity_gains',
            'velocity_filter_alpha',
            'friction_compensation_enabled',
            'friction_model_yaml',
            'motor_types',
            'friction_gain',
            'friction_gains',
            'friction_deadband',
        ]
        vals = self._call_get_parameters(controller_name, names)
        by_name = dict(zip(names, vals))

        joints = list(by_name['joints'].string_array_value)
        n = len(joints)
        gravity_gains = list(by_name['gravity_gains'].double_array_value)
        if not gravity_gains:
            gravity_gains = [1.0] * n
        if len(gravity_gains) != n:
            raise RuntimeError('gravity_gains length does not match joints')

        scalar_friction_gain = float(by_name['friction_gain'].double_value)
        friction_gains = list(by_name['friction_gains'].double_array_value)
        if not friction_gains:
            friction_gains = [scalar_friction_gain] * n
        if len(friction_gains) != n:
            raise RuntimeError('friction_gains length does not match joints')

        motor_types = list(by_name['motor_types'].string_array_value)
        friction_enabled = bool(by_name['friction_compensation_enabled'].bool_value)
        if friction_enabled and len(motor_types) != n:
            raise RuntimeError('motor_types length does not match joints')

        return {
            'joints': joints,
            'gravity_gains': np.asarray(gravity_gains, dtype=float),
            'velocity_filter_alpha': float(by_name['velocity_filter_alpha'].double_value),
            'friction_enabled': friction_enabled,
            'friction_model_yaml': str(by_name['friction_model_yaml'].string_value),
            'motor_types': motor_types,
            'friction_gains': np.asarray(friction_gains, dtype=float),
            'friction_deadband': float(by_name['friction_deadband'].double_value),
        }

    def _load_pinocchio_model(self, robot_description_node: str):
        vals = self._call_get_parameters(robot_description_node, ['robot_description'])
        if not vals or vals[0].type != PARAM_STRING:
            raise RuntimeError(f'/{robot_description_node}/robot_description unavailable')
        xml = vals[0].string_value
        if not xml:
            raise RuntimeError('robot_description is empty')
        try:
            return pin.buildModelFromXML(xml)
        except Exception as exc:
            raise RuntimeError(f'failed to build Pinocchio model from robot_description: {exc}')

    def _build_joint_index_map(self) -> List[int]:
        out = []
        for name in self.joint_names:
            jid = self.model.getJointId(name)
            if jid == 0 or jid >= self.model.njoints:
                raise RuntimeError(f'joint {name} not found in Pinocchio model')
            idx_v = self.model.joints[jid].idx_v
            if idx_v < 0:
                raise RuntimeError(f'joint {name} has invalid idx_v={idx_v}')
            out.append(idx_v)
        return out

    def _load_friction_models(self) -> Dict[str, Dict[str, float]]:
        if not self.params['friction_enabled']:
            return {}
        path = Path(str(self.params['friction_model_yaml'])).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        with path.open() as f:
            data = yaml.safe_load(f) or {}
        models = {}
        for mt in self.params['motor_types']:
            if mt not in data:
                raise RuntimeError(f'motor type {mt} not found in {path}')
            m = data[mt]
            models[mt] = {
                'viscous': float(m['viscous']),
                'coulomb_pos': float(m['coulomb_pos']),
                'coulomb_neg': float(m['coulomb_neg']),
            }
        self.params['friction_model_yaml'] = str(path)
        return models

    # ------------------------------------------------------------------
    # Computation / recording
    # ------------------------------------------------------------------
    def _write_header(self):
        cols = ['stamp', 't']
        for name in self.joint_names:
            cols.extend([
                f'{name}.q',
                f'{name}.v',
                f'{name}.tau_meas',
                f'{name}.tau_gravity',
                f'{name}.tau_friction',
                f'{name}.tau_comp',
                f'{name}.tau_residual',
            ])
        self.csv_writer.writerow(cols)

    def _compute_terms(self, positions, velocities, efforts):
        alpha = self.params['velocity_filter_alpha']
        for i, idx_v in enumerate(self.joint_idx_v):
            self.q[idx_v] = positions[i]
            self.v[idx_v] = velocities[i]
            self.v_filtered[idx_v] = (
                (1.0 - alpha) * velocities[i] + alpha * self.v_filtered[idx_v])

        gravity_full = pin.computeGeneralizedGravity(self.model, self.data, self.q)
        tau_gravity = np.zeros(len(self.joint_names))
        tau_friction = np.zeros(len(self.joint_names))

        for i, idx_v in enumerate(self.joint_idx_v):
            tau_gravity[i] = self.params['gravity_gains'][i] * gravity_full[idx_v]
            if self.params['friction_enabled']:
                v = self.v_filtered[idx_v]
                if abs(v) > self.params['friction_deadband']:
                    mt = self.params['motor_types'][i]
                    fm = self.friction_models[mt]
                    tau_coulomb = fm['coulomb_pos'] if v > 0.0 else -fm['coulomb_neg']
                    tau_friction[i] = (
                        self.params['friction_gains'][i] *
                        (tau_coulomb + fm['viscous'] * v))

        tau_comp = tau_gravity + tau_friction
        tau_residual = efforts - tau_comp
        return tau_gravity, tau_friction, tau_comp, tau_residual

    def _on_joint_state(self, msg: JointState):
        pos_by_name = {n: p for n, p in zip(msg.name, msg.position)}
        vel_by_name = {n: v for n, v in zip(msg.name, msg.velocity)}
        eff_by_name = {n: e for n, e in zip(msg.name, msg.effort)}
        try:
            positions = np.asarray([pos_by_name[n] for n in self.joint_names], dtype=float)
            velocities = np.asarray([vel_by_name[n] for n in self.joint_names], dtype=float)
            efforts = np.asarray([eff_by_name[n] for n in self.joint_names], dtype=float)
        except KeyError:
            return

        tau_g, tau_f, tau_c, tau_r = self._compute_terms(positions, velocities, efforts)
        now = time.monotonic()
        t = now - self.start_mono
        stamp = self.start_wall + t

        with self.lock:
            self.latest = {
                't': t,
                'positions': positions,
                'velocities': velocities,
                'efforts': efforts,
                'gravity': tau_g,
                'friction': tau_f,
                'comp': tau_c,
                'residual': tau_r,
            }
            if now - self.last_record_mono >= 1.0 / self.args.record_rate:
                self.last_record_mono = now
                row = [f'{stamp:.6f}', f'{t:.6f}']
                for i in range(len(self.joint_names)):
                    row.extend([
                        f'{positions[i]:.9g}',
                        f'{velocities[i]:.9g}',
                        f'{efforts[i]:.9g}',
                        f'{tau_g[i]:.9g}',
                        f'{tau_f[i]:.9g}',
                        f'{tau_c[i]:.9g}',
                        f'{tau_r[i]:.9g}',
                    ])
                self.csv_writer.writerow(row)
                self.sample_count += 1
                if self.sample_count % max(1, int(self.args.record_rate)) == 0:
                    self.csv_fh.flush()

            if now - self.last_plot_mono >= 1.0 / self.args.plot_rate:
                self.last_plot_mono = now
                for i, name in enumerate(self.joint_names):
                    self.history[name].append((t, tau_r[i], efforts[i], tau_c[i]))

    def close(self):
        self.csv_fh.flush()
        self.csv_fh.close()


def _parse_joint_selection(raw: str, joint_names: List[str]) -> List[str]:
    raw = (raw or '').strip()
    if not raw:
        return joint_names
    requested = [x.strip() for x in raw.replace(',', ' ').split() if x.strip()]
    missing = [x for x in requested if x not in joint_names]
    if missing:
        raise ValueError(f'plot joints not in controller joints: {missing}')
    return requested


def _setup_plot(node: ForceObserver):
    plot_joints = _parse_joint_selection(node.args.plot_joints, node.joint_names)
    cols = 2 if len(plot_joints) > 1 else 1
    rows = int(math.ceil(len(plot_joints) / cols))
    fig, axes = plt.subplots(rows, cols, squeeze=False, sharex=True)
    axes_flat = axes.flatten()
    lines = {}
    for ax, joint in zip(axes_flat, plot_joints):
        line_res, = ax.plot([], [], label='residual', color='tab:red')
        line_meas, = ax.plot([], [], label='measured', color='tab:blue', alpha=0.35)
        line_comp, = ax.plot([], [], label='comp', color='tab:green', alpha=0.35)
        ax.set_title(joint)
        ax.set_ylabel('Nm')
        ax.grid(True, alpha=0.3)
        lines[joint] = (line_res, line_meas, line_comp, ax)
    for ax in axes_flat[len(plot_joints):]:
        ax.set_visible(False)
    axes_flat[0].legend(loc='upper right')
    fig.suptitle('Torque residual: measured - gravity - friction')
    fig.tight_layout()
    return fig, lines, plot_joints


def _update_plot(node: ForceObserver, lines, plot_joints):
    with node.lock:
        history = {
            joint: list(node.history[joint])
            for joint in plot_joints
        }
    for joint in plot_joints:
        samples = history[joint]
        if not samples:
            continue
        t = [s[0] for s in samples]
        residual = [s[1] for s in samples]
        measured = [s[2] for s in samples]
        comp = [s[3] for s in samples]
        line_res, line_meas, line_comp, ax = lines[joint]
        line_res.set_data(t, residual)
        line_meas.set_data(t, measured)
        line_comp.set_data(t, comp)
        ax.relim()
        ax.autoscale_view()


def parse_args(argv):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', default='recordings/force_observer.csv',
                   help='CSV output path')
    p.add_argument('--joint-state-topic', default='/joint_states')
    p.add_argument('--gravity-controller', default='gravity_compensation_controller')
    p.add_argument('--robot-description-node', default='robot_state_publisher')
    p.add_argument('--record-rate', type=float, default=100.0,
                   help='CSV write rate, Hz')
    p.add_argument('--plot-rate', type=float, default=20.0,
                   help='plot update rate, Hz')
    p.add_argument('--window', type=float, default=10.0,
                   help='plot time window, seconds')
    p.add_argument('--plot-joints', default='',
                   help='optional comma/space separated joints to plot; default all')
    p.add_argument('--no-plot', action='store_true',
                   help='record CSV only')
    args = p.parse_args(argv)
    if args.record_rate <= 0.0:
        p.error('--record-rate must be > 0')
    if args.plot_rate <= 0.0:
        p.error('--plot-rate must be > 0')
    if args.window <= 0.0:
        p.error('--window must be > 0')
    return args


def main():
    argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parse_args(argv)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ForceObserver(args)
    stop = {'value': False}

    def on_signal(_signum, _frame):
        stop['value'] = True

    prev_sigint = signal.signal(signal.SIGINT, on_signal)
    prev_sigterm = signal.signal(signal.SIGTERM, on_signal)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if args.no_plot:
            while rclpy.ok() and not stop['value']:
                time.sleep(0.1)
        else:
            fig, lines, plot_joints = _setup_plot(node)
            plt.ion()
            while rclpy.ok() and not stop['value'] and plt.fignum_exists(fig.number):
                _update_plot(node, lines, plot_joints)
                plt.pause(0.05)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        signal.signal(signal.SIGINT, prev_sigint)
        signal.signal(signal.SIGTERM, prev_sigterm)


if __name__ == '__main__':
    main()
