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
