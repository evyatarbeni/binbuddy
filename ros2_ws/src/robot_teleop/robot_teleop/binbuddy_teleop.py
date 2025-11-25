#!/usr/bin/env python3
"""
BinBuddy custom keyboard teleop with crane controls
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from robot_interfaces.srv import CraneCommand
import sys
import select
import termios
import tty

HELP_MSG = """
BinBuddy Keyboard Teleop
------------------------
Movement:
  W/↑ : Forward
  S/↓ : Backward
  A/← : Turn Left
  D/→ : Turn Right
  SPACE: Stop

Crane:
  H : Home crane
  1 : Stow position
  2 : Pick position
  3 : Place position
  U/I : Manual up/down

Camera:
  C : Cycle cameras (USB → Pi1 → Pi2)

Safety:
  E : Enable movement
  Q : Disable movement
  ESC : Emergency stop

Speed:
  + : Increase speed
  - : Decrease speed

Ctrl+C to quit
"""

class BinBuddyTeleop(Node):
    def __init__(self):
        super().__init__('binbuddy_teleop')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, '/movement/enable', 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)
        self.camera_switch_pub = self.create_publisher(String, '/camera/switch', 10)
        
        # Service client
        self.crane_client = self.create_client(CraneCommand, '/crane/command')
        
        # State
        self.linear_speed = 0.3
        self.angular_speed = 0.8
        self.speed_increment = 0.1
        self.movement_enabled = False
        self.cameras = ['usb', 'picam1', 'picam2']
        self.current_camera_idx = 0
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('BinBuddy teleop started')
        print(HELP_MSG)
    
    def get_key(self, timeout=0.1):
        """Non-blocking key read"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def send_velocity(self, linear, angular):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
    
    def call_crane_service(self, command, target=0.0):
        """Call crane command service"""
        if not self.crane_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Crane service not available')
            return
        
        request = CraneCommand.Request()
        request.command = command
        request.target_position = target
        
        future = self.crane_client.call_async(request)
        future.add_done_callback(self.crane_response_callback)
    
    def crane_response_callback(self, future):
        """Handle crane service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Crane: {response.message}')
            else:
                self.get_logger().warn(f'Crane: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Crane service error: {e}')
    
    def run(self):
        """Main teleop loop"""
        try:
            while True:
                key = self.get_key()
                
                if key == '\x03':  # Ctrl+C
                    break
                elif key == '\x1b':  # ESC
                    msg = Bool()
                    msg.data = True
                    self.estop_pub.publish(msg)
                    self.send_velocity(0.0, 0.0)
                    print("\n🛑 EMERGENCY STOP")
                    
                elif key.lower() == 'w' or key == '\x1b[A':  # W or Up arrow
                    self.send_velocity(self.linear_speed, 0.0)
                elif key.lower() == 's' or key == '\x1b[B':  # S or Down arrow
                    self.send_velocity(-self.linear_speed, 0.0)
                elif key.lower() == 'a' or key == '\x1b[D':  # A or Left arrow
                    self.send_velocity(0.0, self.angular_speed)
                elif key.lower() == 'd' or key == '\x1b[C':  # D or Right arrow
                    self.send_velocity(0.0, -self.angular_speed)
                elif key == ' ':  # Space
                    self.send_velocity(0.0, 0.0)
                    
                elif key.lower() == 'e':
                    self.movement_enabled = True
                    msg = Bool()
                    msg.data = True
                    self.enable_pub.publish(msg)
                    print("✓ Movement ENABLED")
                elif key.lower() == 'q':
                    self.movement_enabled = False
                    msg = Bool()
                    msg.data = False
                    self.enable_pub.publish(msg)
                    self.send_velocity(0.0, 0.0)
                    print("✗ Movement DISABLED")
                    
                elif key.lower() == 'h':
                    print("Homing crane...")
                    self.call_crane_service("HOME")
                elif key == '1':
                    print("Crane → STOW")
                    self.call_crane_service("STOW")
                elif key == '2':
                    print("Crane → PICK")
                    self.call_crane_service("PICK")
                elif key == '3':
                    print("Crane → PLACE")
                    self.call_crane_service("PLACE")
                    
                elif key.lower() == 'c':
                    self.current_camera_idx = (self.current_camera_idx + 1) % len(self.cameras)
                    camera = self.cameras[self.current_camera_idx]
                    msg = String()
                    msg.data = camera
                    self.camera_switch_pub.publish(msg)
                    print(f"📷 Camera: {camera}")
                    
                elif key == '+' or key == '=':
                    self.linear_speed = min(1.0, self.linear_speed + self.speed_increment)
                    self.angular_speed = min(2.0, self.angular_speed + self.speed_increment)
                    print(f"Speed: {self.linear_speed:.1f} m/s")
                elif key == '-' or key == '_':
                    self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
                    self.angular_speed = max(0.2, self.angular_speed - self.speed_increment)
                    print(f"Speed: {self.linear_speed:.1f} m/s")
                
        except Exception as e:
            self.get_logger().error(f'Teleop error: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.send_velocity(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = BinBuddyTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
