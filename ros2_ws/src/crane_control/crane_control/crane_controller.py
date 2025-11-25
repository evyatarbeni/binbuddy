#!/usr/bin/env python3
"""
Crane controller with action server, limit switches, and preset positions
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from robot_interfaces.msg import CraneState
from robot_interfaces.action import MoveCrane
from robot_interfaces.srv import CraneCommand
from std_msgs.msg import Bool
import serial
import time
from threading import Lock

class CraneController(Node):
    def __init__(self):
        super().__init__('crane_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('position_stow', 0.0)
        self.declare_parameter('position_pick', 75.0)
        self.declare_parameter('position_place', 100.0)
        self.declare_parameter('homing_speed', 30.0)
        self.declare_parameter('default_speed', 50.0)
        self.declare_parameter('movement_enabled', False)
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.pos_stow = self.get_parameter('position_stow').value
        self.pos_pick = self.get_parameter('position_pick').value
        self.pos_place = self.get_parameter('position_place').value
        self.homing_speed = self.get_parameter('homing_speed').value
        self.default_speed = self.get_parameter('default_speed').value
        
        # State
        self.movement_enabled = self.get_parameter('movement_enabled').value
        self.current_position = 0.0
        self.homed = False
        self.at_lower_limit = False
        self.at_upper_limit = False
        self.moving = False
        self.current_preset = "STOW"
        self.serial_lock = Lock()
        self.arduino = None
        
        # Serial connection
        self.connect_serial()
        
        # Publishers
        self.state_pub = self.create_publisher(CraneState, '/crane/state', 10)
        
        # Subscribers
        self.enable_sub = self.create_subscription(
            Bool, '/movement/enable', self.enable_callback, 10)
        
        # Services
        self.command_srv = self.create_service(
            CraneCommand, '/crane/command', self.command_callback)
        
        # Action server
        self.action_server = ActionServer(
            self, MoveCrane, '/crane/move',
            execute_callback=self.execute_move,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        # Timers
        self.create_timer(0.1, self.update_state)  # 10 Hz
        self.create_timer(0.2, self.publish_state)  # 5 Hz
        
        # Auto-home disabled by default (uncomment to enable)
        # self.create_timer(1.0, self.auto_home_once, clock=None)
        
        self.get_logger().info('Crane controller initialized (auto-home disabled)')
    
    def connect_serial(self):
        """Connect to Arduino (shared with base if same port)"""
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            time.sleep(2)
            self.get_logger().info(f'Crane connected to {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Crane serial error: {e}')
            self.arduino = None
    
    def auto_home_once(self):
        """Auto-home on first call, then cancel timer"""
        if not self.homed:
            self.get_logger().info('Auto-homing crane...')
            self.home_crane()
    
    def enable_callback(self, msg):
        """Movement enable/disable"""
        self.movement_enabled = msg.data
        if not self.movement_enabled:
            self.stop_crane()
    
    def command_callback(self, request, response):
        """Service to execute crane commands"""
        cmd = request.command.upper()
        
        if not self.movement_enabled and cmd != "HOME":
            response.success = False
            response.message = "Movement disabled"
            return response
        
        if cmd == "HOME":
            success = self.home_crane()
            response.success = success
            response.message = "Homed" if success else "Homing failed"
            response.final_position = self.current_position
            
        elif cmd == "STOW":
            self.move_to_position(self.pos_stow)
            self.current_preset = "STOW"
            response.success = True
            response.message = "Moving to stow"
            response.final_position = self.pos_stow
            
        elif cmd == "PICK":
            self.move_to_position(self.pos_pick)
            self.current_preset = "PICK"
            response.success = True
            response.message = "Moving to pick"
            response.final_position = self.pos_pick
            
        elif cmd == "PLACE":
            self.move_to_position(self.pos_place)
            self.current_preset = "PLACE"
            response.success = True
            response.message = "Moving to place"
            response.final_position = self.pos_place
            
        elif cmd == "GOTO":
            target = max(0.0, min(100.0, request.target_position))
            self.move_to_position(target)
            self.current_preset = "CUSTOM"
            response.success = True
            response.message = f"Moving to {target}%"
            response.final_position = target
            
        else:
            response.success = False
            response.message = f"Unknown command: {cmd}"
        
        return response
    
    def goal_callback(self, goal_request):
        """Accept or reject action goals"""
        if not self.movement_enabled:
            self.get_logger().warn('Goal rejected - movement disabled')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle action cancellation"""
        self.get_logger().info('Goal cancelled')
        self.stop_crane()
        return CancelResponse.ACCEPT
    
    async def execute_move(self, goal_handle):
        """Execute MoveCrane action"""
        target = max(0.0, min(100.0, goal_handle.request.target_position))
        speed = max(10.0, min(100.0, goal_handle.request.speed))
        
        self.get_logger().info(f'Moving crane to {target}% at {speed}% speed')
        self.move_to_position(target, speed)
        
        # Simulate movement with feedback
        feedback = MoveCrane.Feedback()
        start_pos = self.current_position
        
        # Simple open-loop simulation (replace with encoder feedback if available)
        steps = 20
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                self.stop_crane()
                goal_handle.canceled()
                result = MoveCrane.Result()
                result.success = False
                result.final_position = self.current_position
                return result
            
            # Linear interpolation
            progress = (i + 1) / steps
            self.current_position = start_pos + (target - start_pos) * progress
            feedback.current_position = self.current_position
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)
        
        self.current_position = target
        goal_handle.succeed()
        
        result = MoveCrane.Result()
        result.success = True
        result.final_position = self.current_position
        return result
    
    def home_crane(self):
        """Home crane to lower limit switch"""
        if not self.arduino:
            return False
        
        self.get_logger().info('Homing crane...')
        try:
            # Protocol: [0xFF, 0xAA, cmd=0x10, speed, checksum]
            speed_byte = int(self.homing_speed)
            checksum = (0x10 + speed_byte) & 0xFF
            packet = bytes([0xFF, 0xAA, 0x10, speed_byte, checksum])
            
            with self.serial_lock:
                self.arduino.write(packet)
            
            # Wait for homing (timeout 10s)
            timeout = time.time() + 10.0
            while time.time() < timeout:
                if self.at_lower_limit:
                    self.homed = True
                    self.current_position = 0.0
                    self.current_preset = "STOW"
                    self.get_logger().info('Homing complete')
                    return True
                time.sleep(0.1)
            
            self.get_logger().error('Homing timeout')
            return False
            
        except Exception as e:
            self.get_logger().error(f'Homing error: {e}')
            return False
    
    def move_to_position(self, position, speed=None):
        """Move crane to absolute position (0-100%)"""
        if not self.arduino:
            return
        
        speed = speed if speed else self.default_speed
        self.moving = True
        
        try:
            # Protocol: [0xFF, 0xAA, cmd=0x11, pos, speed, checksum]
            pos_byte = int(position)
            speed_byte = int(speed)
            checksum = (0x11 + pos_byte + speed_byte) & 0xFF
            packet = bytes([0xFF, 0xAA, 0x11, pos_byte, speed_byte, checksum])
            
            with self.serial_lock:
                self.arduino.write(packet)
                
        except Exception as e:
            self.get_logger().error(f'Move error: {e}')
            self.moving = False
    
    def stop_crane(self):
        """Emergency stop crane"""
        if not self.arduino:
            return
        
        try:
            # Protocol: [0xFF, 0xAA, cmd=0x12, checksum]
            packet = bytes([0xFF, 0xAA, 0x12, 0x12])
            with self.serial_lock:
                self.arduino.write(packet)
            self.moving = False
        except Exception as e:
            self.get_logger().error(f'Stop error: {e}')
    
    def update_state(self):
        """Read limit switches and update state from Arduino"""
        if not self.arduino:
            return
        
        try:
            # Read limit switch status from Arduino
            # Protocol expects: [0xFF, 0xBB, lower_sw, upper_sw, checksum]
            with self.serial_lock:
                if self.arduino.in_waiting >= 4:
                    data = self.arduino.read(4)
                    if len(data) == 4 and data[0] == 0xFF and data[1] == 0xBB:
                        self.at_lower_limit = bool(data[2])
                        self.at_upper_limit = bool(data[3])
                        
        except Exception as e:
            self.get_logger().error(f'State read error: {e}', throttle_duration_sec=5.0)
    
    def publish_state(self):
        """Publish crane state"""
        state = CraneState()
        state.position_percent = self.current_position
        state.homed = self.homed
        state.at_lower_limit = self.at_lower_limit
        state.at_upper_limit = self.at_upper_limit
        state.moving = self.moving
        state.current_preset = self.current_preset
        self.state_pub.publish(state)

def main(args=None):
    rclpy.init(args=args)
    node = CraneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_crane()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
