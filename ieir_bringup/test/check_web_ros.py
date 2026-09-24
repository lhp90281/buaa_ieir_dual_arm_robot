#!/usr/bin/env python3
"""Fake ROS graph integration. Isolated domain; never open CAN or enable motors."""
import os
os.environ['ROS_DOMAIN_ID'] = '218'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ieir_bringup/scripts'))
from web_console_core import ConsoleState, GRAVITY, JOINT, CARTESIAN, robot_model
from web_console_ros import RosBackend
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory
from w3_robot_bridge.msg import MotorState, MotorStateArray


def wait_for(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError('Timed out waiting for fake ROS state')


def main():
    model, _ = robot_model(ROOT / 'description')
    state = ConsoleState(model)
    backend = RosBackend(state)
    bus = Node('web_console_fake_robot')
    backend.executor.add_node(bus)
    modes = {GRAVITY: 'active', JOINT: 'inactive', CARTESIAN: 'inactive'}
    received, switches = [], []
    publish_feedback, error = True, 1
    right_position = 0.0
    joints = bus.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
    motors = bus.create_publisher(MotorStateArray, '/w3_robot_bridge_node/state', qos_profile_sensor_data)
    tf = TransformBroadcaster(bus)
    def feedback():
        if not publish_feedback:
            return
        message = JointState()
        message.header.stamp = bus.get_clock().now().to_msg()
        message.name = list(state.limits)
        message.position = [0.0] * 14
        message.position[message.name.index('right_joint_0')] = right_position
        message.velocity = [0.0] * 14
        message.effort = [0.0] * 14
        joints.publish(message)
        array = MotorStateArray()
        for name in state.limits:
            item = MotorState()
            item.channel = int(name.startswith('right'))
            item.motor_index = int(name[-1])
            item.online, item.enabled, item.error_flags = True, True, error
            array.motors.append(item)
        motors.publish(array)
        for side in ('left', 'right'):
            transform = TransformStamped()
            transform.header.stamp = message.header.stamp
            transform.header.frame_id = 'base_footprint'
            transform.child_frame_id = side + '_attachment_point'
            transform.transform.translation.z = 1.0
            transform.transform.rotation.w = 1.0
            tf.sendTransform(transform)
    bus.create_timer(0.05, feedback)
    def listing(request, response):
        response.controller = [ControllerState(name=n, state=s) for n, s in modes.items()]
        return response
    def switching(request, response):
        assert GRAVITY not in request.deactivate_controllers
        switches.append(request)
        for name in request.activate_controllers:
            modes[name] = 'active'
        for name in request.deactivate_controllers:
            modes[name] = 'inactive'
        response.ok = True
        return response
    def teleop(request, response):
        response.success, response.message = True, 'fake teleop acknowledged'
        return response
    list_service = bus.create_service(ListControllers, '/controller_manager/list_controllers', listing)
    bus.create_service(SwitchController, '/controller_manager/switch_controller', switching)
    bus.create_service(Trigger, '/teleop/disable', teleop)
    bus.create_subscription(JointTrajectory, '/joint_position_command', received.append, 10)
    poses = []
    bus.create_subscription(PoseStamped, '/cartesian_position_controller/right_target_pose', poses.append, 10)
    try:
        wait_for(lambda: state.controller_at > 0 and len(state.joints) == 14 and 'right' in state.poses)
        assert not switches and not received and not poses, 'Attach must never send control'
        backend.execute('switch', dict(mode='joint'))
        wait_for(lambda: state.controllers.get(JOINT) == 'active' and time.monotonic() - state.controller_at < 3)
        backend.execute('joint', dict(positions={'right_joint_0': 0.1}, duration=3))
        wait_for(lambda: len(received) == 1)
        assert received[0].joint_names == ['right_joint_0']
        assert received[0].points[0].time_from_start.sec == 3
        error = 2
        wait_for(lambda: state.motors and state.motors[0]['error'] == 2)
        try:
            backend.execute('joint', dict(positions={'right_joint_0': 0.1}, duration=3))
            raise AssertionError('Motor fault must block motion')
        except ValueError as exc:
            assert 'healthy' in str(exc)
        error = 1
        wait_for(lambda: state.motors and state.motors[0]['error'] == 1)
        backend.execute('switch', dict(mode='cartesian'))
        wait_for(lambda: state.controllers.get(CARTESIAN) == 'active' and time.monotonic() - state.controller_at < 3)
        backend.execute('cartesian', dict(side='right', position=[.01, 0, 1], quaternion=[0, 0, 0, 1]))
        wait_for(lambda: len(poses) == 1)
        assert poses[0].header.frame_id == 'base_footprint'
        right_position = 0.8
        wait_for(lambda: state.joints['right_joint_0']['position'] == 0.8)
        backend.execute('zero', dict(side='right', duration=8))
        wait_for(lambda: len(received) == 2)
        zero = received[-1]
        assert zero.joint_names == [f'right_joint_{i}' for i in range(7)]
        assert zero.points[0].positions[0] == 0.8 and zero.points[0].time_from_start.sec == 0
        assert list(zero.points[-1].positions) == [0.0] * 7
        assert zero.points[-1].time_from_start.sec == 8
        assert len(zero.points) == 401
        assert modes[JOINT] == modes[GRAVITY] == 'active' and modes[CARTESIAN] == 'inactive'
        other = bus.create_publisher(JointTrajectory, '/joint_position_command', 10)
        wait_for(lambda: len(backend.get_publishers_info_by_topic('/joint_position_command')) == 2)
        try:
            backend.execute('zero', dict(side='dual', duration=8))
            raise AssertionError('An external trajectory owner must block zero')
        except ValueError as exc:
            assert 'Another target publisher' in str(exc)
        bus.destroy_publisher(other)
        assert backend.execute('teleop', dict(operation='disable')) == 'fake teleop acknowledged'
        publish_feedback = False
        time.sleep(0.7)
        try:
            backend.execute('cartesian', dict(side='right', position=[.01, 0, 1], quaternion=[0, 0, 0, 1]))
            raise AssertionError('Stale feedback must block motion')
        except ValueError as exc:
            assert 'Stale' in str(exc)
        try:
            backend.execute('zero', dict(side='right', duration=8))
            raise AssertionError('Stale feedback must block zero')
        except ValueError as exc:
            assert 'Stale' in str(exc)
        assert len(received) == 2 and len(poses) == 1
        bus.destroy_service(list_service)
        wait_for(lambda: not state.controllers and state.controller_at == 0)
        print('PASS: attach-only, zero ramp/scope/switch/ownership, gravity retained, joint/pose publication, TF, fault/stale feedback gates, teleop service')
    finally:
        backend.executor.remove_node(bus)
        bus.destroy_node()
        backend.close()


if __name__ == '__main__':
    main()
