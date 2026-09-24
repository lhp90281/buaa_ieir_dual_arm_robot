#!/usr/bin/env python3
"""
Cartesian Keyboard Control for Dual Arm Robot

Position control:
  W/S : +/- X    A/D : +/- Y    Q/E : +/- Z

Rotation control (numpad with NumLock ON):
  8/2 : +/- Pitch (Y-axis)    4/6 : +/- Roll (X-axis)    7/9 : +/- Yaw (Z-axis)

Arm selection:
  Tab : Cycle (Left -> Right -> Both)

Other:
  +/- : Step size    R : Joint homing    ESC/Ctrl+C : Exit

Keyboard focus must be on the terminal running this script.
Press and hold to move continuously, release to stop.
"""

import sys
import os
import math
import tty
import termios
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
# Attempt to import pinocchio for FK (optional, for display)
try:
    import pinocchio
    HAS_PINOCCHIO = True
except ImportError:
    HAS_PINOCCHIO = False


class CartesianKeyboardController(Node):
    def __init__(self):
        super().__init__('cartesian_keyboard_controller')

        # Parameters
        self.step_size = 0.005  # meters per update cycle
        self.rot_step = 0.02   # radians per update cycle (~1.15 deg)
        self.update_rate = 50.0  # Hz
        self.active_arm = 'left'  # 'left', 'right', 'both'

        # Current target positions (will be initialized from initial EE pose)
        self.left_pos = [0.0, 0.0, 0.0]
        self.right_pos = [0.0, 0.0, 0.0]

        # Current orientation as quaternion [x, y, z, w]
        self.left_orient = [0.0, 0.0, 0.0, 1.0]
        self.right_orient = [0.0, 0.0, 0.0, 1.0]

        # Initial poses (saved for reset)
        self.left_init_pos = None
        self.left_init_orient = None
        self.right_init_pos = None
        self.right_init_orient = None

        # Initial pose received flag
        self.left_initialized = False
        self.right_initialized = False

        # Translation movement state
        self.move_x = 0  # -1, 0, +1
        self.move_y = 0
        self.move_z = 0

        # Rotation movement state
        self.move_roll = 0   # -1, 0, +1  (around X)
        self.move_pitch = 0  # -1, 0, +1  (around Y)
        self.move_yaw = 0    # -1, 0, +1  (around Z)

        # Homing state
        self.homing_active = False
        self.homing_arm = ''  # which arm is homing: 'left'/'right'/'both'

        # Publishers
        self.left_pub = self.create_publisher(
            PoseStamped,
            '/cartesian_position_controller/left_target_pose', 10)
        self.right_pub = self.create_publisher(
            PoseStamped,
            '/cartesian_position_controller/right_target_pose', 10)
        self.homing_pub = self.create_publisher(
            String,
            '/cartesian_position_controller/joint_homing', 10)

        # Subscribe to joint states to compute initial EE pose
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.homing_status_sub = self.create_subscription(
            String, '/cartesian_position_controller/homing_status',
            self.homing_status_callback, 10)
        # Subscribe to homing FK poses (exact EE pose after homing)
        self.left_homing_fk_sub = self.create_subscription(
            PoseStamped, '/cartesian_position_controller/left_homing_fk',
            self.left_homing_fk_callback, 10)
        self.right_homing_fk_sub = self.create_subscription(
            PoseStamped, '/cartesian_position_controller/right_homing_fk',
            self.right_homing_fk_callback, 10)

        self.initial_joint_state_received = False
        self.joint_positions = {}

        # Timer for continuous publishing
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)

        self.running = True

        self.get_logger().info('Cartesian Keyboard Controller started')
        self.get_logger().info(f'Step size: {self.step_size:.4f} m, Update rate: {self.update_rate} Hz')

    def left_homing_fk_callback(self, msg):
        """Receive exact FK pose from controller after left arm homing."""
        p = msg.pose.position
        o = msg.pose.orientation
        self.left_pos = [p.x, p.y, p.z]
        self.left_orient = [o.x, o.y, o.z, o.w]
        self.get_logger().info(
            f'Left arm FK updated: [{p.x:.4f}, {p.y:.4f}, {p.z:.4f}]')

    def right_homing_fk_callback(self, msg):
        """Receive exact FK pose from controller after right arm homing."""
        p = msg.pose.position
        o = msg.pose.orientation
        self.right_pos = [p.x, p.y, p.z]
        self.right_orient = [o.x, o.y, o.z, o.w]
        self.get_logger().info(
            f'Right arm FK updated: [{p.x:.4f}, {p.y:.4f}, {p.z:.4f}]')

    def homing_status_callback(self, msg):
        """Handle homing status from controller."""
        if msg.data == 'done':
            # Homing complete - FK poses are updated via homing_fk callbacks
            self.homing_active = False
            self.homing_arm = ''
        else:
            # Homing started for arm(s)
            self.homing_active = True
            self.homing_arm = msg.data

    def joint_state_callback(self, msg):
        """Receive joint states to initialize EE positions."""
        if self.initial_joint_state_received:
            return

        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

        # Check if we have all arm joints
        left_joints = [f'left_joint_{i}' for i in range(7)]
        right_joints = [f'right_joint_{i}' for i in range(7)]

        if all(j in self.joint_positions for j in left_joints + right_joints):
            self.initial_joint_state_received = True
            self.get_logger().info('Received initial joint states')

    def set_initial_pose(self, arm, pos, orient):
        """Set initial EE pose for an arm."""
        if arm == 'left':
            self.left_pos = list(pos)
            self.left_orient = list(orient)
            self.left_init_pos = list(pos)
            self.left_init_orient = list(orient)
            self.left_initialized = True
            self.get_logger().info(
                f'Left arm initial pos: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}], '
                f'orient: [{orient[0]:.4f}, {orient[1]:.4f}, {orient[2]:.4f}, {orient[3]:.4f}]')
        elif arm == 'right':
            self.right_pos = list(pos)
            self.right_orient = list(orient)
            self.right_init_pos = list(pos)
            self.right_init_orient = list(orient)
            self.right_initialized = True
            self.get_logger().info(
                f'Right arm initial pos: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}], '
                f'orient: [{orient[0]:.4f}, {orient[1]:.4f}, {orient[2]:.4f}, {orient[3]:.4f}]')

    @staticmethod
    def _quat_multiply(q1, q2):
        """Multiply two quaternions [x,y,z,w] format."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return [
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
        ]

    @staticmethod
    def _quat_normalize(q):
        """Normalize quaternion [x,y,z,w]."""
        n = math.sqrt(sum(c*c for c in q))
        if n < 1e-12:
            return [0.0, 0.0, 0.0, 1.0]
        return [c / n for c in q]

    def _apply_rotation(self, orient, roll, pitch, yaw):
        """Apply incremental rotation (world frame) to quaternion [x,y,z,w]."""
        # Build small rotation quaternions for each axis
        # Roll  = rotation around X
        # Pitch = rotation around Y
        # Yaw   = rotation around Z
        result = list(orient)
        if abs(roll) > 1e-9:
            hr = roll / 2.0
            dq = [math.sin(hr), 0.0, 0.0, math.cos(hr)]
            result = self._quat_multiply(dq, result)  # world frame: dq * q
        if abs(pitch) > 1e-9:
            hp = pitch / 2.0
            dq = [0.0, math.sin(hp), 0.0, math.cos(hp)]
            result = self._quat_multiply(dq, result)
        if abs(yaw) > 1e-9:
            hy = yaw / 2.0
            dq = [0.0, 0.0, math.sin(hy), math.cos(hy)]
            result = self._quat_multiply(dq, result)
        return self._quat_normalize(result)

    def timer_callback(self):
        """Publish target poses at fixed rate."""
        if not (self.left_initialized and self.right_initialized):
            return

        # Translation deltas
        dx = self.move_x * self.step_size
        dy = self.move_y * self.step_size
        dz = self.move_z * self.step_size

        # Rotation deltas
        d_roll = self.move_roll * self.rot_step
        d_pitch = self.move_pitch * self.rot_step
        d_yaw = self.move_yaw * self.rot_step

        # Skip publishing for arm(s) that are currently homing
        left_homing = self.homing_arm in ('left', 'both')
        right_homing = self.homing_arm in ('right', 'both')

        if self.active_arm in ('left', 'both') and not left_homing:
            self.left_pos[0] += dx
            self.left_pos[1] += dy
            self.left_pos[2] += dz
            if d_roll != 0 or d_pitch != 0 or d_yaw != 0:
                self.left_orient = self._apply_rotation(
                    self.left_orient, d_roll, d_pitch, d_yaw)
            self._publish_pose(self.left_pub, self.left_pos, self.left_orient)

        if self.active_arm in ('right', 'both') and not right_homing:
            self.right_pos[0] += dx
            self.right_pos[1] += dy
            self.right_pos[2] += dz
            if d_roll != 0 or d_pitch != 0 or d_yaw != 0:
                self.right_orient = self._apply_rotation(
                    self.right_orient, d_roll, d_pitch, d_yaw)
            self._publish_pose(self.right_pub, self.right_pos, self.right_orient)

    def _publish_pose(self, pub, pos, orient):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.x = orient[0]
        msg.pose.orientation.y = orient[1]
        msg.pose.orientation.z = orient[2]
        msg.pose.orientation.w = orient[3]
        pub.publish(msg)

def get_initial_ee_poses(node):
    """
    Wait for joint states and compute initial EE poses using Pinocchio.
    Falls back to hardcoded initial poses if Pinocchio is not available.
    """
    # Wait for joint states
    print("Waiting for /joint_states ...")
    timeout = 10.0
    start = time.time()
    while not node.initial_joint_state_received:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start > timeout:
            print("Timeout waiting for /joint_states. Using default initial poses.")
            # Default poses from controller log
            node.set_initial_pose('left',  [-0.2055, 0.0, 0.6702], [0.0, 0.0, 0.0, 1.0])
            node.set_initial_pose('right', [ 0.2055, 0.0, 0.6702], [0.0, 0.0, 0.0, 1.0])
            return

    if HAS_PINOCCHIO:
        try:
            # Build Pinocchio model from URDF
            from ament_index_python.packages import get_package_share_directory
            import numpy as np

            urdf_path = os.path.join(
                get_package_share_directory('dual_arm_support'),
                'urdf', 'dual_arm_robot_plug.urdf')

            model = pinocchio.buildModelFromUrdf(urdf_path)
            data = model.createData()

            # Fill q vector
            q = pinocchio.neutral(model)
            for j_id in range(1, model.njoints):
                j_name = model.names[j_id]
                if j_name in node.joint_positions:
                    idx_v = model.joints[j_id].idx_v
                    q[idx_v] = node.joint_positions[j_name]

            pinocchio.forwardKinematics(model, data, q)
            pinocchio.updateFramePlacements(model, data)

            for frame_name, arm in [('left_link_7', 'left'), ('right_link_7', 'right')]:
                if model.existFrame(frame_name):
                    fid = model.getFrameId(frame_name)
                    pose = data.oMf[fid]
                    pos = pose.translation.tolist()
                    quat = pinocchio.Quaternion(pose.rotation)
                    orient = [float(quat.x), float(quat.y), float(quat.z), float(quat.w)]
                    node.set_initial_pose(arm, pos, orient)
                else:
                    print(f"Frame {frame_name} not found, using default")
                    if arm == 'left':
                        node.set_initial_pose('left', [-0.2055, 0.0, 0.6702], [0.0, 0.0, 0.0, 1.0])
                    else:
                        node.set_initial_pose('right', [0.2055, 0.0, 0.6702], [0.0, 0.0, 0.0, 1.0])
            return
        except Exception as e:
            print(f"Pinocchio FK failed: {e}. Using default poses.")

    # Fallback: use logged initial poses
    node.set_initial_pose('left',  [-0.2055, 0.0, 0.6702], [0.0, 0.0, 0.0, 1.0])
    node.set_initial_pose('right', [ 0.2055, 0.0, 0.6702], [0.0, 0.0, 0.0, 1.0])


def print_status(node):
    """Print current status to terminal."""
    arm_str = {'left': 'LEFT', 'right': 'RIGHT', 'both': 'BOTH'}[node.active_arm]
    homing_str = ' [HOMING]' if node.homing_active else ''
    sys.stdout.write(
        f'\r  [{arm_str}] pos:{node.step_size:.4f}m rot:{math.degrees(node.rot_step):.1f}deg  '
        f'L:[{node.left_pos[0]:+.3f},{node.left_pos[1]:+.3f},{node.left_pos[2]:+.3f}]  '
        f'R:[{node.right_pos[0]:+.3f},{node.right_pos[1]:+.3f},{node.right_pos[2]:+.3f}]  '
        f'T:[{node.move_x:+d},{node.move_y:+d},{node.move_z:+d}] '
        f'Rot:[{node.move_roll:+d},{node.move_pitch:+d},{node.move_yaw:+d}]{homing_str}    ')
    sys.stdout.flush()


def keyboard_loop(node):
    """Main keyboard reading loop using raw terminal input."""
    old_settings = termios.tcgetattr(sys.stdin)

    print("\n" + "=" * 70)
    print("  Cartesian Keyboard Control")
    print("=" * 70)
    print("  Position:  W/S: +/-X    A/D: +/-Y    Q/E: +/-Z")
    print("  Rotation:  8/2: +/-Pitch  4/6: +/-Roll  7/9: +/-Yaw  (numpad)")
    print("  Arm:       Tab: cycle (Left/Right/Both)")
    print("  Other:     +/-: Step size    R: Joint homing    ESC: Exit")
    print("=" * 70)

    try:
        tty.setraw(sys.stdin.fileno())

        # Tap-and-hold style: a key sets motion direction; if no key event arrives
        # within key_timeout, motion stops.
        last_key_time = 0.0
        key_timeout = 0.15

        while node.running:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.02)

            if rlist:
                ch = sys.stdin.read(1)
                last_key_time = time.time()

                # ESC
                if ch == '\x1b':
                    node.running = False
                    break
                # Ctrl+C
                elif ch == '\x03':
                    node.running = False
                    break

                ch_lower = ch.lower()

                # --- Position keys ---
                if ch_lower == 'w':
                    node.move_x = 1
                elif ch_lower == 's':
                    node.move_x = -1
                elif ch_lower == 'a':
                    node.move_y = 1
                elif ch_lower == 'd':
                    node.move_y = -1
                elif ch_lower == 'q':
                    node.move_z = 1
                elif ch_lower == 'e':
                    node.move_z = -1
                # --- Rotation keys (numpad digits) ---
                elif ch == '8':
                    node.move_pitch = 1
                elif ch == '2':
                    node.move_pitch = -1
                elif ch == '4':
                    node.move_roll = -1
                elif ch == '6':
                    node.move_roll = 1
                elif ch == '7':
                    node.move_yaw = 1
                elif ch == '9':
                    node.move_yaw = -1
                # --- Arm selection (Tab to cycle) ---
                elif ch == '\t':
                    cycle = {'left': 'right', 'right': 'both', 'both': 'left'}
                    node.active_arm = cycle[node.active_arm]
                # --- Step size ---
                elif ch in ('+', '='):
                    node.step_size = min(node.step_size * 1.5, 0.05)
                    node.rot_step = min(node.rot_step * 1.5, 0.2)
                elif ch in ('-', '_'):
                    node.step_size = max(node.step_size / 1.5, 0.0005)
                    node.rot_step = max(node.rot_step / 1.5, 0.002)
                elif ch_lower == 'r':
                    # Joint-level homing for active arm
                    msg = String()
                    msg.data = node.active_arm
                    node.homing_pub.publish(msg)
                    node.move_x = 0
                    node.move_y = 0
                    node.move_z = 0
                    node.move_roll = 0
                    node.move_pitch = 0
                    node.move_yaw = 0
                print_status(node)
            else:
                # No key pressed within timeout - stop motion
                if time.time() - last_key_time > key_timeout:
                    has_motion = (node.move_x != 0 or node.move_y != 0 or node.move_z != 0
                                  or node.move_roll != 0 or node.move_pitch != 0 or node.move_yaw != 0)
                    if has_motion:
                        node.move_x = 0
                        node.move_y = 0
                        node.move_z = 0
                        node.move_roll = 0
                        node.move_pitch = 0
                        node.move_yaw = 0
                        print_status(node)

            rclpy.spin_once(node, timeout_sec=0)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\n\nExiting keyboard control.")


def main():
    rclpy.init()
    node = CartesianKeyboardController()

    try:
        # Get initial EE poses
        get_initial_ee_poses(node)

        # Send initial pose once to establish baseline
        if node.left_initialized:
            node._publish_pose(node.left_pub, node.left_pos, node.left_orient)
        if node.right_initialized:
            node._publish_pose(node.right_pub, node.right_pos, node.right_orient)

        # Start keyboard control
        keyboard_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
