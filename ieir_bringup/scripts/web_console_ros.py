"""Attach-only ROS adapter: no launch, shell, enable service or raw CAN command."""
import math
import json
import os
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
from w3_robot_bridge.msg import MotorStateArray

from web_console_core import GRAVITY, JOINT, CARTESIAN, switch_plan, zero_trajectory


class RosBackend(Node):
    def __init__(self, state):
        rclpy.init(args=[])
        super().__init__(f'ieir_web_console_{os.getpid()}')
        self.state = state
        self.preview_subs, self.key_publishers = {}, {}
        self.controller_future = None
        self.controller_requested_at = 0.0
        self.browser_seen_at = 0.0
        self.joint_pub = None
        self.pose_pubs = {}
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(MotorStateArray, '/w3_robot_bridge_node/state', self.on_motors,
                                 qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self.on_log, qos_profile_sensor_data)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.create_timer(1.0, self.refresh_graph)
        self.create_timer(0.1, self.refresh_poses)
        self.create_timer(0.5, self.preview_heartbeat)
        self.console_executor = MultiThreadedExecutor(num_threads=2)
        self.console_executor.add_node(self)
        self.thread = threading.Thread(target=self.console_executor.spin, daemon=True)
        self.thread.start()
        self.state.log('Attached to existing ROS graph; no nodes or motors started')

    def on_joints(self, msg):
        now = time.monotonic()
        with self.state.lock:
            for i, name in enumerate(msg.name):
                if name not in self.state.limits or i >= len(msg.position):
                    continue
                q = float(msg.position[i])
                if not math.isfinite(q):
                    self.state.joints.pop(name, None)
                    continue
                v = float(msg.velocity[i]) if i < len(msg.velocity) else 0.0
                effort = float(msg.effort[i]) if i < len(msg.effort) else 0.0
                self.state.joints[name] = dict(position=q, velocity=v if math.isfinite(v) else None,
                                               effort=effort if math.isfinite(effort) else None, at=now)

    def on_motors(self, msg):
        with self.state.lock:
            now = time.monotonic()
            self.state.motors = [dict(channel=m.channel, slot=m.motor_index, online=m.online,
                                      enabled=m.enabled, position=float(m.position),
                                      velocity=float(m.velocity), torque=float(m.torque),
                                      error=m.error_flags, at=now) for m in msg.motors
                                 if m.channel in (0, 1)]

    def on_log(self, msg):
        level = 'error' if msg.level >= 40 else 'warn' if msg.level >= 30 else 'info'
        if msg.level >= 20:
            self.state.log(msg.msg, level, msg.name)

    def refresh_poses(self):
        for side in ('left', 'right'):
            try:
                tf = self.tf.lookup_transform('base_footprint', f'{side}_attachment_point', Time())
                # Latest TF must be recent, not merely still present in the buffer.
                age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
                if not -0.1 <= age <= 0.5:
                    continue
                p, q = tf.transform.translation, tf.transform.rotation
                with self.state.lock:
                    self.state.poses[side] = dict(position=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w],
                                                  at=time.monotonic())
            except Exception:
                pass  # Snapshot retains the last value, whose age expires.

    def refresh_graph(self):
        with self.state.lock:
            self.state.services = [n for n, _ in self.get_service_names_and_types()]
            self.state.nodes = sorted(self.get_node_names())
        if not self.list_client.service_is_ready():
            with self.state.lock:
                self.state.controllers = {}
                self.state.controller_at = 0.0
            if self.controller_future is not None:
                self.controller_future.cancel()
                self.controller_future = None
        if (self.controller_future is not None and not self.controller_future.done()
                and time.monotonic() - self.controller_requested_at > 3):
            self.controller_future.cancel()
            self.controller_future = None
        if self.controller_future is not None and self.controller_future.done():
            try:
                response = self.controller_future.result()
                with self.state.lock:
                    self.state.controllers = {c.name: c.state for c in response.controller}
                    self.state.controller_at = time.monotonic()
            except Exception as exc:
                self.state.log(str(exc), 'warn')
            self.controller_future = None
        if self.controller_future is None and self.list_client.service_is_ready():
            self.controller_future = self.list_client.call_async(ListControllers.Request())
            self.controller_requested_at = time.monotonic()
        prefixes = {name[:-7] for name, types in self.get_topic_names_and_types()
                    if name.startswith('/calibration_preview_') and name.endswith('/status')
                    and 'std_msgs/msg/String' in types and self.count_publishers(name) > 0}
        for prefix in prefixes - self.preview_subs.keys():
            self.preview_subs[prefix] = [
                self.create_subscription(String, prefix + '/status',
                                         lambda msg, p=prefix: self.on_preview_status(p, msg), 10),
                self.create_subscription(JointState, prefix + '/pose',
                                         lambda msg, p=prefix: self.on_preview_pose(p, msg), 10),
                self.create_subscription(String, prefix + '/metadata',
                                         lambda msg, p=prefix: self.on_preview_metadata(p, msg), 10)]
        for prefix in list(self.preview_subs.keys() - prefixes):
            for sub in self.preview_subs.pop(prefix):
                self.destroy_subscription(sub)
            with self.state.lock:
                self.state.calibration.pop(prefix, None)
            with self.state.lock:
                if prefix in self.key_publishers:
                    self.destroy_publisher(self.key_publishers.pop(prefix))

    def on_preview_status(self, prefix, msg):
        with self.state.lock:
            self.state.calibration.setdefault(prefix, {}).update(text=msg.data[:1600], at=time.monotonic())

    def on_preview_pose(self, prefix, msg):
        with self.state.lock:
            self.state.calibration.setdefault(prefix, {}).update(
                positions={name: value for name, value in zip(msg.name, msg.position)
                           if name in self.state.limits and math.isfinite(value)})

    def on_preview_metadata(self, prefix, msg):
        try:
            metadata = json.loads(msg.data)
            if not isinstance(metadata, dict):
                return
        except ValueError:
            return
        with self.state.lock:
            self.state.calibration.setdefault(prefix, {}).update(metadata=metadata)

    def key_publisher(self, prefix):
        # Called by the executor and HTTP thread; creation must be serialized.
        with self.state.lock:
            if prefix not in self.key_publishers:
                self.key_publishers[prefix] = self.create_publisher(String, prefix + '/key', 10)
            return self.key_publishers[prefix]

    def preview_heartbeat(self):
        if time.monotonic() - self.browser_seen_at > 1.5:
            return
        with self.state.lock:
            prefixes = [p for p, v in self.state.calibration.items()
                        if v.get('metadata', {}).get('backend') == 'web'
                        and time.monotonic() - v.get('at', 0) < 2]
        for prefix in prefixes:
            self.key_publisher(prefix).publish(String(data='ui_heartbeat'))

    def check_idle_graph(self):
        if self.list_client.service_is_ready() or 'controller_manager' in self.get_node_names():
            raise ValueError('Stop the controller stack first; external processes are not managed by this UI')
        for topic in ('/w3_robot_bridge_node/commands', '/w3_robot_bridge_node/command'):
            if self.count_publishers(topic):
                raise ValueError(f'Another motor command publisher exists: {topic}')
        with self.state.lock:
            if any(time.monotonic() - v.get('at', 0) < 2 for v in self.state.calibration.values()):
                raise ValueError('Another calibration session is active')
            motors = list(self.state.motors)
        if self.count_publishers('/w3_robot_bridge_node/state') != 1 or not any(
                time.monotonic() - m['at'] < .5 for m in motors):
            raise ValueError('Exactly one live W3 bridge is required')
        if any(m['enabled'] or m['error'] == 1 for m in motors):
            raise ValueError('Motors are still enabled; verify controller/calibration shutdown and disable first')

    def check_target_start(self, kind):
        self.check_motor_feedback(self.state.fresh_joints())
        self.state.require_controller(GRAVITY)
        if any(n.startswith('/teleop/') for n, _ in self.get_service_names_and_types()):
            raise ValueError('An external teleop node already owns the target interfaces')
        if kind == 'teleop':
            self.state.fresh_joints([f'{side}_joint_{i}' for side in ('left', 'right') for i in range(7)])
        for topic in ('/joint_position_command', '/cartesian_position_controller/left_target_pose',
                      '/cartesian_position_controller/right_target_pose'):
            self.check_no_other_target_publisher(topic)
        if self.joint_pub is not None:
            self.destroy_publisher(self.joint_pub)
            self.joint_pub = None
        for publisher in self.pose_pubs.values():
            self.destroy_publisher(publisher)
        self.pose_pubs.clear()

    def call(self, service_type, path, request, timeout=8):
        client = self.create_client(service_type, path)
        try:
            if not client.wait_for_service(timeout_sec=0.5):
                raise ValueError(f'Service unavailable: {path}')
            future = client.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(timeout):
                raise ValueError(f'{path}: response timed out; outcome unknown. Do not retry automatically.')
            return future.result()
        finally:
            self.destroy_client(client)

    def check_no_other_target_publisher(self, topic):
        others = [p.node_name for p in self.get_publishers_info_by_topic(topic)
                  if p.node_name != self.get_name() or p.node_namespace != self.get_namespace()]
        if others:
            raise ValueError(f'Another target publisher owns {topic}: {", ".join(others)}')

    @staticmethod
    def wait_for_subscriber(publisher):
        deadline = time.monotonic() + 1
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        if publisher.get_subscription_count() == 0:
            raise ValueError('No controller target subscriber')

    def check_motor_feedback(self, names):
        with self.state.lock:
            motors = {(m['channel'], m['slot']): m for m in self.state.motors}
            for name in names:
                channel = 0 if name.startswith('left_') else 1
                slot = int(name.rsplit('_', 1)[1])
                motor = motors.get((channel, slot))
                if (not motor or not motor['online'] or not motor['enabled'] or motor['error'] != 1
                        or time.monotonic() - motor['at'] > 0.5
                        or not all(math.isfinite(motor[k]) for k in ('position', 'velocity', 'torque'))):
                    raise ValueError(f'W3 feedback is not healthy/enabled: {name}')

    def execute(self, action, data):
        if action not in ('calibration_key', 'calibration_angle', 'teleop'):
            with self.state.lock:
                if any(time.monotonic() - v.get('at', 0) < 2 for v in self.state.calibration.values()):
                    raise ValueError('Close active calibration before using arm control')
        if action == 'zero':
            current, _ = self.state.zero_plan(data)
            self.check_motor_feedback(current)
            self.execute('switch', dict(mode='joint'))
            # Do not publish until the manager confirms the actual active controllers.
            response = self.call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
            with self.state.lock:
                self.state.controllers = {c.name: c.state for c in response.controller}
                self.state.controller_at = time.monotonic()
            self.state.require_controller(JOINT)
            self.state.require_controller(GRAVITY)
            if self.joint_pub is None:
                self.joint_pub = self.create_publisher(JointTrajectory, '/joint_position_command', 10)
            self.wait_for_subscriber(self.joint_pub)
            current, duration = self.state.zero_plan(data)
            self.check_motor_feedback(current)
            self.check_no_other_target_publisher('/joint_position_command')
            message = JointTrajectory()
            message.joint_names = list(current)
            for seconds, positions in zero_trajectory(current, duration):
                point = JointTrajectoryPoint()
                point.positions = positions
                nanoseconds = round(seconds * 1e9)
                point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 10**9)
                message.points.append(point)
            self.joint_pub.publish(message)
            return f'Model-zero trajectory published ({data["side"]}, {duration:.2f} s); arrival is not yet confirmed'
        if action == 'switch':
            self.check_motor_feedback(self.state.fresh_joints())
            self.check_no_other_target_publisher('/joint_position_command')
            for side in ('left', 'right'):
                self.check_no_other_target_publisher(f'/cartesian_position_controller/{side}_target_pose')
            response = self.call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
            add, remove = switch_plan({c.name: c.state for c in response.controller}, data.get('mode'))
            if not add and not remove:
                return 'Requested controller mode is already active'
            request = SwitchController.Request()
            request.activate_controllers, request.deactivate_controllers = add, remove
            request.strictness, request.activate_asap = request.STRICT, False
            request.timeout.sec = 5
            result = self.call(SwitchController, '/controller_manager/switch_controller', request)
            if not result.ok:
                raise ValueError('Controller switch rejected')
            with self.state.lock:
                self.state.controller_at = 0  # Require a new manager acknowledgment before targets.
            return 'Controller manager accepted switch; waiting for refreshed state'
        if action == 'joint':
            targets, duration = self.state.joint_target(data)
            self.check_motor_feedback(targets)
            self.check_no_other_target_publisher('/joint_position_command')
            message = JointTrajectory()
            message.joint_names = list(targets)
            point = JointTrajectoryPoint()
            point.positions = [float(targets[n]) for n in message.joint_names]
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration - int(duration))*1e9)
            message.points = [point]
            if self.joint_pub is None:
                self.joint_pub = self.create_publisher(JointTrajectory, '/joint_position_command', 10)
            self.wait_for_subscriber(self.joint_pub)
            self.state.joint_target(data)
            self.check_motor_feedback(targets)
            self.joint_pub.publish(message)
            return 'Joint trajectory published; arrival is not yet confirmed'
        if action == 'cartesian':
            side, p, q = self.state.cartesian_target(data)
            self.check_motor_feedback([f'{side}_joint_{i}' for i in range(7)])
            self.check_no_other_target_publisher(f'/cartesian_position_controller/{side}_target_pose')
            message = PoseStamped()
            message.header.frame_id = 'base_footprint'
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose.position.x, message.pose.position.y, message.pose.position.z = p
            (message.pose.orientation.x, message.pose.orientation.y,
             message.pose.orientation.z, message.pose.orientation.w) = q
            if side not in self.pose_pubs:
                self.pose_pubs[side] = self.create_publisher(PoseStamped,
                    f'/cartesian_position_controller/{side}_target_pose', 10)
            self.wait_for_subscriber(self.pose_pubs[side])
            self.state.cartesian_target(data)
            self.check_motor_feedback([f'{side}_joint_{i}' for i in range(7)])
            self.pose_pubs[side].publish(message)
            return 'Cartesian target published; IK success/arrival is not acknowledged by this interface'
        if action == 'teleop':
            operation = data.get('operation')
            if operation not in ('prepare', 'enable', 'disable', 'exit'):
                raise ValueError('Unknown teleop operation')
            if operation in ('prepare', 'enable'):
                self.check_motor_feedback(self.state.fresh_joints())
                if self.state.calibration:
                    raise ValueError('Close calibration before teleoperation')
            result = self.call(Trigger, '/teleop/' + operation, Trigger.Request(), timeout=45)
            if not result.success:
                raise ValueError(result.message)
            return result.message
        if action in ('calibration_key', 'calibration_angle'):
            prefix, key = data.get('session'), data.get('key')
            if action == 'calibration_key' and key not in ('enter', 'space', 'n', 's', 'r', 'backspace', 'escape'):
                raise ValueError('Unknown calibration key')
            item = self.state.calibration.get(prefix)
            if not item or time.monotonic() - item.get('at', 0) > 2:
                raise ValueError('Calibration session is no longer current')
            if self.list_client.service_is_ready():
                raise ValueError('Stop controllers before calibration')
            if action == 'calibration_angle':
                angle = data.get('angle_deg')
                if item.get('metadata', {}).get('stage') != 'limits':
                    raise ValueError('Angle editing is only available during limit review')
                if type(angle) not in (float, int) or not math.isfinite(angle) or not -360 <= angle <= 360:
                    raise ValueError('Reference angle must be finite and within +/-360 degrees')
                key = f'angle_deg:{angle}'
            publisher = self.key_publisher(prefix)
            deadline = time.monotonic() + 1
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                time.sleep(0.02)
            if publisher.get_subscription_count() == 0:
                raise ValueError('No calibration key subscriber')
            message = String()
            message.data = key
            publisher.publish(message)
            return f'Calibration key sent: {key}'
        raise ValueError('Unknown command')

    def close(self):
        self.console_executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.destroy_node()
        rclpy.try_shutdown()
