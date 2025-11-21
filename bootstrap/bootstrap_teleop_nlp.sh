#!/bin/bash
###############################################################################
# Generate robot_teleop and robot_nlp packages
# Contains: Keyboard teleop and ChatGPT command interface
###############################################################################

set -e
WORKSPACE_SRC="${1:-$(pwd)}"
cd "$WORKSPACE_SRC"

echo "→ Generating robot_teleop and robot_nlp..."

###############################################################################
# ROBOT_TELEOP
###############################################################################
mkdir -p robot_teleop/{robot_teleop,resource}
touch robot_teleop/resource/robot_teleop
touch robot_teleop/robot_teleop/__init__.py

# package.xml
cat > robot_teleop/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>robot_teleop</name>
  <version>1.0.0</version>
  <description>Custom keyboard teleop for BinBuddy</description>
  <maintainer email="evyatarbeni@gmail.com">Evyatar Beni</maintainer>
  <license>MIT</license>
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>robot_interfaces</depend>
  <export><build_type>ament_python</build_type></export>
</package>
EOF

# setup.py
cat > robot_teleop/setup.py << 'EOF'
from setuptools import setup, find_packages

setup(
    name='robot_teleop',
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_teleop']),
        ('share/robot_teleop', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evyatar Beni',
    maintainer_email='evyatarbeni@gmail.com',
    description='Custom keyboard teleop',
    license='MIT',
    entry_points={
        'console_scripts': [
            'binbuddy_teleop = robot_teleop.binbuddy_teleop:main',
        ],
    },
)
EOF

# setup.cfg
cat > robot_teleop/setup.cfg << 'EOF'
[develop]
script_dir=$base/lib/robot_teleop
[install]
install_scripts=$base/lib/robot_teleop
EOF

# Teleop node
cat > robot_teleop/robot_teleop/binbuddy_teleop.py << 'EOFPYTHON'
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
EOFPYTHON

###############################################################################
# ROBOT_NLP
###############################################################################
mkdir -p robot_nlp/{robot_nlp,resource}
touch robot_nlp/resource/robot_nlp
touch robot_nlp/robot_nlp/__init__.py

# Create .env file
cat > robot_nlp/.env << 'EOF'
# OpenAI API Configuration
OPENAI_API_KEY=your-openai-api-key-here
EOF

# package.xml
cat > robot_nlp/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>robot_nlp</name>
  <version>1.0.0</version>
  <description>Natural language command interface using ChatGPT</description>
  <maintainer email="evyatarbeni@gmail.com">Evyatar Beni</maintainer>
  <license>MIT</license>
  <depend>rclpy</depend>
  <depend>robot_interfaces</depend>
  <export><build_type>ament_python</build_type></export>
</package>
EOF

# setup.py
cat > robot_nlp/setup.py << 'EOF'
from setuptools import setup, find_packages

setup(
    name='robot_nlp',
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_nlp']),
        ('share/robot_nlp', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evyatar Beni',
    maintainer_email='evyatarbeni@gmail.com',
    description='NLP command interface',
    license='MIT',
    entry_points={
        'console_scripts': [
            'nlp_commander = robot_nlp.nlp_commander:main',
        ],
    },
)
EOF

# setup.cfg
cat > robot_nlp/setup.cfg << 'EOF'
[develop]
script_dir=$base/lib/robot_nlp
[install]
install_scripts=$base/lib/robot_nlp
EOF

# NLP commander node
cat > robot_nlp/robot_nlp/nlp_commander.py << 'EOFPYTHON'
#!/usr/bin/env python3
"""
Natural language command processor using OpenAI ChatGPT
Converts English commands to ROS 2 action sequences
"""
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import NLPCommand, CraneCommand, SetMode
from geometry_msgs.msg import PoseStamped
import os
from dotenv import load_dotenv

# Try to import openai
try:
    import openai
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

class NLPCommander(Node):
    def __init__(self):
        super().__init__('nlp_commander')
        
        # Load environment variables
        load_dotenv()
        api_key = os.getenv('OPENAI_API_KEY')
        
        if not OPENAI_AVAILABLE:
            self.get_logger().error('OpenAI not installed: pip install openai')
            self.enabled = False
        elif not api_key or api_key == 'your-openai-api-key-here':
            self.get_logger().error('OpenAI API key not set in .env file')
            self.enabled = False
        else:
            openai.api_key = api_key
            self.enabled = True
            self.get_logger().info('NLP commander initialized with OpenAI')
        
        # Service
        self.nlp_srv = self.create_service(
            NLPCommand, '/nlp/command', self.process_command)
        
        # Service clients (for executing commands)
        self.crane_client = self.create_client(CraneCommand, '/crane/command')
        self.mode_client = self.create_client(SetMode, '/mode/set')
    
    def process_command(self, request, response):
        """Process natural language command"""
        if not self.enabled:
            response.success = False
            response.error_message = "NLP not available - check API key"
            return response
        
        command_text = request.command_text
        self.get_logger().info(f'Processing: "{command_text}"')
        
        try:
            # Call ChatGPT to interpret command
            interpretation = self.interpret_command(command_text)
            
            # Parse and execute
            actions = self.parse_actions(interpretation)
            
            response.success = True
            response.interpretation = interpretation
            response.planned_actions = actions
            response.error_message = ""
            
            # Execute actions (async)
            self.execute_actions(actions)
            
        except Exception as e:
            self.get_logger().error(f'NLP error: {e}')
            response.success = False
            response.error_message = str(e)
        
        return response
    
    def interpret_command(self, text):
        """Use ChatGPT to interpret command"""
        system_prompt = """You are a robot command interpreter. Convert natural language 
commands into simple action sequences for a differential drive robot with a crane.

Available actions:
- MOVE_TO(location): Navigate to named location
- CRANE_HOME: Home the crane
- CRANE_STOW: Stow crane for travel
- CRANE_PICK: Lower crane to pick position
- CRANE_PLACE: Raise crane to place position
- WAIT(seconds): Wait for specified time

Example:
Input: "go to bin A, pick it up, deliver to zone B, then return home"
Output: MOVE_TO(bin_A), CRANE_PICK, WAIT(2), CRANE_STOW, MOVE_TO(zone_B), 
CRANE_PLACE, WAIT(2), CRANE_STOW, MOVE_TO(home)

Respond with only the action sequence, no explanation."""

        try:
            completion = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": text}
                ],
                max_tokens=200,
                temperature=0.3
            )
            return completion.choices[0].message.content.strip()
        except Exception as e:
            self.get_logger().error(f'OpenAI API error: {e}')
            raise
    
    def parse_actions(self, interpretation):
        """Parse action sequence from ChatGPT response"""
        actions = []
        for part in interpretation.split(','):
            action = part.strip()
            if action:
                actions.append(action)
        return actions
    
    def execute_actions(self, actions):
        """Execute action sequence (simplified)"""
        self.get_logger().info(f'Executing {len(actions)} actions...')
        for action in actions:
            self.get_logger().info(f'  → {action}')
            # Actual execution would call appropriate services/actions
            # This is a placeholder showing the architecture

def main(args=None):
    rclpy.init(args=args)
    node = NLPCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOFPYTHON

echo "  ✓ robot_teleop and robot_nlp created"
