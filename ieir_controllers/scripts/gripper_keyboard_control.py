#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class GripperKeyboardControl(Node):
    def __init__(self):
        super().__init__('gripper_keyboard_control')
        
        # Publishers for both grippers
        self.left_pub = self.create_publisher(String, '/gripper_controller/left_gripper/command', 10)
        self.right_pub = self.create_publisher(String, '/gripper_controller/right_gripper/command', 10)
        
        # Store original terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("Gripper Keyboard Control Started")
        self.get_logger().info("Controls:")
        self.get_logger().info("  p - Close gripper (hold to keep closing)")
        self.get_logger().info("  o - Open gripper (hold to keep opening)")
        self.get_logger().info("  1 - Control left gripper only")
        self.get_logger().info("  2 - Control right gripper only")
        self.get_logger().info("  3 - Control both grippers (default)")
        self.get_logger().info("  q - Quit")
        
        # Control mode: 'left', 'right', or 'both'
        self.control_mode = 'both'
        
        # Current key state
        self.current_key = None
        self.key_pressed = False
        
    def get_key(self):
        """Blocking key reading with timeout"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def send_command(self, command):
        """Send command to gripper(s) based on control mode"""
        msg = String()
        msg.data = command
        
        if self.control_mode == 'left' or self.control_mode == 'both':
            self.left_pub.publish(msg)
        if self.control_mode == 'right' or self.control_mode == 'both':
            self.right_pub.publish(msg)
    
    def run(self):
        """Main control loop"""
        import threading
        
        running = True
        
        def key_listener():
            nonlocal running
            while running:
                try:
                    key = self.get_key()
                    
                    if key == 'q':
                        self.get_logger().info("Quitting...")
                        running = False
                        break
                    elif key == '1':
                        self.control_mode = 'left'
                        self.get_logger().info("Control mode: LEFT gripper only")
                    elif key == '2':
                        self.control_mode = 'right'
                        self.get_logger().info("Control mode: RIGHT gripper only")
                    elif key == '3':
                        self.control_mode = 'both'
                        self.get_logger().info("Control mode: BOTH grippers")
                    elif key == 'p':
                        if not self.key_pressed or self.current_key != 'p':
                            self.send_command('close')
                            self.get_logger().info(f"Closing {self.control_mode} gripper(s)")
                            self.current_key = 'p'
                            self.key_pressed = True
                    elif key == 'o':
                        if not self.key_pressed or self.current_key != 'o':
                            self.send_command('open')
                            self.get_logger().info(f"Opening {self.control_mode} gripper(s)")
                            self.current_key = 'o'
                            self.key_pressed = True
                    else:
                        # Any other key releases the hold
                        if self.key_pressed:
                            self.send_command('hold')
                            self.get_logger().info(f"Holding {self.control_mode} gripper(s)")
                            self.key_pressed = False
                            self.current_key = None
                            
                except Exception as e:
                    if running:
                        self.get_logger().error(f"Key listener error: {e}")
                    break
        
        # Start key listener thread
        listener_thread = threading.Thread(target=key_listener, daemon=True)
        listener_thread.start()
        
        try:
            while running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                
        except KeyboardInterrupt:
            self.get_logger().info("Interrupted by user")
            running = False
        finally:
            running = False
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # Send hold command before exit
            try:
                self.send_command('hold')
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = GripperKeyboardControl()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
