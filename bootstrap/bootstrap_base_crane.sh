#!/bin/bash
###############################################################################
# Generate robot_base and crane_control packages
# Contains: Motor control, odometry, crane positioning
###############################################################################

set -e
WORKSPACE_SRC="${1:-$(pwd)}"
cd "$WORKSPACE_SRC"

echo "→ Generating robot_base and crane_control..."

###############################################################################
# ROBOT_BASE PACKAGE
###############################################################################
mkdir -p robot_base/{robot_base,resource}
touch robot_base/resource/robot_base
touch robot_base/robot_base/__init__.py

cat > robot_base/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>robot_base</name>
  <version>1.0.0</version>
  <description>Motor control and odometry for BinBuddy</description>
  <maintainer email="evyatarbeni@gmail.com">Evyatar Beni</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>robot_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

cat > robot_base/setup.py << 'EOF'
from setuptools import setup, find_packages

package_name = 'robot_base'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evyatar Beni',
    maintainer_email='evyatarbeni@gmail.com',
    description='Motor control and odometry for BinBuddy base',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_controller = robot_base.base_controller:main',
        ],
    },
)
EOF

cat > robot_base/setup.cfg << 'EOF'
[develop]
script_dir=$base/lib/robot_base
[install]
install_scripts=$base/lib/robot_base
EOF

cat > robot_base/robot_base/base_controller.py << 'EOFPYTHON'
#!/usr/bin/env python3
"""
Base controller node - bridges ROS 2 cmd_vel to Arduino motor control
Publishes odometry and manages safety watchdog
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from robot_interfaces.msg import SystemHealth
from tf2_ros import TransformBroadcaster
import serial
import time
import math
from threading import Lock

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.35)
        self.declare_parameter('wheel_radius', 0.075)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('movement_enabled', False)
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        
        # State
        self.movement_enabled = self.get_parameter('movement_enabled').value
        self.last_cmd_time = time.time()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.serial_lock = Lock()
        self.arduino = None
        
        # Serial connection
        self.connect_serial()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.health_pub = self.create_publisher(SystemHealth, '/system/health', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.enable_sub = self.create_subscription(
            Bool, '/movement/enable', self.enable_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, '/estop', self.estop_callback, 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timers
        self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.create_timer(0.05, self.publish_odometry)  # 20 Hz
        self.create_timer(1.0, self.publish_health)  # 1 Hz
        
        self.get_logger().info('Base controller initialized')
        
    def connect_serial(self):
        """Establish serial connection to Arduino"""
        try:
            self.arduino = serial.Serial(
                self.serial_port, self.baud_rate, timeout=0.1)
            time.sleep(2)  # Wait for Arduino reset
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.arduino = None
    
    def cmd_vel_callback(self, msg):
        """Receive velocity commands"""
        if self.movement_enabled:
            self.vx = max(-self.max_linear, min(self.max_linear, msg.linear.x))
            self.vth = max(-self.max_angular, min(self.max_angular, msg.angular.z))
            self.last_cmd_time = time.time()
        else:
            self.get_logger().warn('Movement disabled - ignoring cmd_vel', throttle_duration_sec=2.0)
    
    def enable_callback(self, msg):
        """Enable/disable movement"""
        self.movement_enabled = msg.data
        if not self.movement_enabled:
            self.send_stop()
            self.get_logger().info('Movement disabled - robot stopped')
        else:
            self.get_logger().info('Movement enabled')
    
    def estop_callback(self, msg):
        """Emergency stop"""
        if msg.data:
            self.movement_enabled = False
            self.send_stop()
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
    
    def control_loop(self):
        """Main control loop with watchdog"""
        # Watchdog: stop if no commands received
        if time.time() - self.last_cmd_time > self.watchdog_timeout:
            if self.vx != 0.0 or self.vth != 0.0:
                self.vx = 0.0
                self.vth = 0.0
                self.send_stop()
        
        # Send commands to Arduino
        if self.movement_enabled:
            self.send_velocity(self.vx, self.vth)
        
        # Update odometry (dead reckoning)
        dt = 0.02  # 50 Hz
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
    
    def send_velocity(self, linear, angular):
        """Send velocity command to Arduino"""
        if not self.arduino:
            return
        
        try:
            # Protocol: [0xFF, 0xAA, cmd=0x01, vx_low, vx_high, vth_low, vth_high, checksum]
            # Scale to int16: -1000 to 1000
            vx_scaled = int(linear * 1000)
            vth_scaled = int(angular * 1000)
            
            vx_low = vx_scaled & 0xFF
            vx_high = (vx_scaled >> 8) & 0xFF
            vth_low = vth_scaled & 0xFF
            vth_high = (vth_scaled >> 8) & 0xFF
            
            checksum = (0x01 + vx_low + vx_high + vth_low + vth_high) & 0xFF
            
            packet = bytes([0xFF, 0xAA, 0x01, vx_low, vx_high, vth_low, vth_high, checksum])
            
            with self.serial_lock:
                self.arduino.write(packet)
                
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def send_stop(self):
        """Emergency stop command"""
        if not self.arduino:
            return
            
        try:
            # Protocol: [0xFF, 0xAA, cmd=0x02, checksum]
            packet = bytes([0xFF, 0xAA, 0x02, 0x02])
            with self.serial_lock:
                self.arduino.write(packet)
        except Exception as e:
            self.get_logger().error(f'Stop command error: {e}')
    
    def publish_odometry(self):
        """Publish odometry message and TF"""
        now = self.get_clock().now().to_msg()
        
        # Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Quaternion from yaw
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom)
        
        # TF transform
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_health(self):
        """Publish system health status"""
        health = SystemHealth()
        health.arduino_connected = self.arduino is not None and self.arduino.is_open
        health.movement_enabled = self.movement_enabled
        self.health_pub.publish(health)

def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOFPYTHON

###############################################################################
# CRANE_CONTROL PACKAGE
###############################################################################
mkdir -p crane_control/{crane_control,resource}
touch crane_control/resource/crane_control
touch crane_control/crane_control/__init__.py

cat > crane_control/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>crane_control</name>
  <version>1.0.0</version>
  <description>Crane position control with action server</description>
  <maintainer email="evyatarbeni@gmail.com">Evyatar Beni</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>robot_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

cat > crane_control/setup.py << 'EOF'
from setuptools import setup, find_packages

package_name = 'crane_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evyatar Beni',
    maintainer_email='evyatarbeni@gmail.com',
    description='Crane position control with action server',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crane_controller = crane_control.crane_controller:main',
        ],
    },
)
EOF

cat > crane_control/setup.cfg << 'EOF'
[develop]
script_dir=$base/lib/crane_control
[install]
install_scripts=$base/lib/crane_control
EOF

cat > crane_control/crane_control/crane_controller.py << 'EOFPYTHON'
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
EOFPYTHON

echo "  ✓ robot_base and crane_control created"
