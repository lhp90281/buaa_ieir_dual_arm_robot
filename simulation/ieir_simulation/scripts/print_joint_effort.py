#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class JointEffortDiffPrinter(Node):
    def __init__(self):
        super().__init__('joint_effort_diff_printer')
        
        self.target_joint = 'left_joint_1'
        self.latest_joint_state = None
        self.latest_ctrl_effort = None
        
        self.sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
            
        self.sub_ctrl_effort = self.create_subscription(
            JointState,
            '/ctrl/effort',
            self.ctrl_effort_callback,
            10)
            
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(f'Monitoring effort difference for joint: {self.target_joint}')

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def ctrl_effort_callback(self, msg):
        self.latest_ctrl_effort = msg

    def get_effort(self, msg, joint_name):
        if msg is None:
            return None
        try:
            idx = msg.name.index(joint_name)
            if idx < len(msg.effort):
                return msg.effort[idx]
        except ValueError:
            pass
        return None

    def timer_callback(self):
        if self.latest_joint_state is None or self.latest_ctrl_effort is None:
            print(f"[{time.strftime('%H:%M:%S')}] Waiting for data...")
            return
        
        measured = self.get_effort(self.latest_joint_state, self.target_joint)
        commanded = self.get_effort(self.latest_ctrl_effort, self.target_joint)
        
        if measured is None:
            print(f"[{time.strftime('%H:%M:%S')}] Joint {self.target_joint} not found in /joint_states")
            return
            
        if commanded is None:
            print(f"[{time.strftime('%H:%M:%S')}] Joint {self.target_joint} not found in /ctrl/effort")
            return

        diff = measured - commanded
        
        # Calculate time diff
        ts_measured = self.latest_joint_state.header.stamp.sec + self.latest_joint_state.header.stamp.nanosec * 1e-9
        ts_commanded = self.latest_ctrl_effort.header.stamp.sec + self.latest_ctrl_effort.header.stamp.nanosec * 1e-9
        time_diff = ts_measured - ts_commanded
        
        print(f"[{time.strftime('%H:%M:%S')}] {self.target_joint} | Measured: {measured:.4f} | Commanded: {commanded:.4f} | Diff: {diff:.4f} | TimeDelta: {time_diff:.4f}s")

def main(args=None):
    rclpy.init(args=args)
    printer = JointEffortDiffPrinter()
    try:
        rclpy.spin(printer)
    except KeyboardInterrupt:
        pass
    finally:
        printer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
