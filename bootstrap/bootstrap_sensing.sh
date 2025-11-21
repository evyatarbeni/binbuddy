#!/bin/bash
###############################################################################
# Generate robot_sensing package
# Contains: Triple camera manager + LiDAR relay
###############################################################################

set -e
WORKSPACE_SRC="${1:-$(pwd)}"
cd "$WORKSPACE_SRC"

echo "→ Generating robot_sensing..."

mkdir -p robot_sensing/{robot_sensing,resource}
touch robot_sensing/resource/robot_sensing
touch robot_sensing/robot_sensing/__init__.py

# package.xml
cat > robot_sensing/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>robot_sensing</name>
  <version>1.0.0</version>
  <description>Camera manager and LiDAR integration</description>
  <maintainer email="evyatarbeni@gmail.com">Evyatar Beni</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>std_msgs</depend>
  <depend>robot_interfaces</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# setup.py
cat > robot_sensing/setup.py << 'EOF'
from setuptools import setup, find_packages

package_name = 'robot_sensing'

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
    description='Camera manager and sensor integration',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_manager = robot_sensing.camera_manager:main',
        ],
    },
)
EOF

# setup.cfg
cat > robot_sensing/setup.cfg << 'EOF'
[develop]
script_dir=$base/lib/robot_sensing
[install]
install_scripts=$base/lib/robot_sensing
EOF

# Camera manager node
cat > robot_sensing/robot_sensing/camera_manager.py << 'EOFPYTHON'
#!/usr/bin/env python3
"""
Triple camera manager - switches between USB and 2x Pi cameras
Handles video recording and snapshot capture
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from robot_interfaces.srv import CameraCommand
from cv_bridge import CvBridge
import cv2
import subprocess
import os
import time
from datetime import datetime

class CameraManager(Node):
    def __init__(self):
        super().__init__('camera_manager')
        
        # Parameters
        self.declare_parameter('default_camera', 'usb')
        self.declare_parameter('usb_device', '/dev/video0')
        self.declare_parameter('picam1_index', 0)
        self.declare_parameter('picam2_index', 1)
        self.declare_parameter('frame_rate', 15)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('recording_dir', os.path.expanduser('~/binbuddy_recordings'))
        
        self.default_camera = self.get_parameter('default_camera').value
        self.usb_device = self.get_parameter('usb_device').value
        self.picam1_idx = self.get_parameter('picam1_index').value
        self.picam2_idx = self.get_parameter('picam2_index').value
        self.fps = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        self.recording_dir = self.get_parameter('recording_dir').value
        
        # Create recording directory
        os.makedirs(self.recording_dir, exist_ok=True)
        
        # State
        self.active_camera = self.default_camera
        self.bridge = CvBridge()
        self.capture = None
        self.recording = False
        self.video_writer = None
        self.current_recording_file = None
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.compressed_pub = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', 10)
        
        # Subscribers
        self.switch_sub = self.create_subscription(
            String, '/camera/switch', self.switch_callback, 10)
        
        # Services
        self.control_srv = self.create_service(
            CameraCommand, '/camera/control', self.control_callback)
        
        # Timer for image capture
        self.create_timer(1.0 / self.fps, self.capture_and_publish)
        
        # Initialize camera
        self.switch_camera(self.active_camera)
        
        self.get_logger().info(f'Camera manager initialized (active: {self.active_camera})')
    
    def switch_callback(self, msg):
        """Handle camera switch requests"""
        camera_name = msg.data.lower()
        self.switch_camera(camera_name)
    
    def switch_camera(self, camera_name):
        """Switch active camera"""
        if camera_name not in ['usb', 'picam1', 'picam2']:
            self.get_logger().warn(f'Invalid camera: {camera_name}')
            return
        
        # Stop recording if active
        if self.recording:
            self.stop_recording()
        
        # Release current capture
        if self.capture:
            self.capture.release()
            self.capture = None
        
        # Open new camera
        try:
            if camera_name == 'usb':
                self.capture = cv2.VideoCapture(self.usb_device)
            elif camera_name == 'picam1':
                self.capture = cv2.VideoCapture(self.picam1_idx)
            elif camera_name == 'picam2':
                self.capture = cv2.VideoCapture(self.picam2_idx)
            
            if self.capture and self.capture.isOpened():
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.capture.set(cv2.CAP_PROP_FPS, self.fps)
                self.active_camera = camera_name
                self.get_logger().info(f'Switched to camera: {camera_name}')
            else:
                self.get_logger().error(f'Failed to open camera: {camera_name}')
                
        except Exception as e:
            self.get_logger().error(f'Camera switch error: {e}')
    
    def control_callback(self, request, response):
        """Handle camera control commands"""
        cmd = request.command.lower()
        cam = request.camera_name.lower() if request.camera_name else self.active_camera
        
        if cmd == 'start_recording':
            if cam != self.active_camera:
                self.switch_camera(cam)
            success, filepath = self.start_recording()
            response.success = success
            response.message = "Recording started" if success else "Failed to start recording"
            response.file_path = filepath if success else ""
            
        elif cmd == 'stop_recording':
            success = self.stop_recording()
            response.success = success
            response.message = "Recording stopped" if success else "Not recording"
            response.file_path = self.current_recording_file if success else ""
            
        elif cmd == 'snapshot':
            if cam != self.active_camera:
                self.switch_camera(cam)
            success, filepath = self.take_snapshot()
            response.success = success
            response.message = "Snapshot saved" if success else "Failed to capture"
            response.file_path = filepath if success else ""
            
        elif cmd == 'switch':
            self.switch_camera(cam)
            response.success = True
            response.message = f"Switched to {cam}"
            response.file_path = ""
            
        else:
            response.success = False
            response.message = f"Unknown command: {cmd}"
            response.file_path = ""
        
        return response
    
    def start_recording(self):
        """Start video recording"""
        if self.recording:
            return False, ""
        
        if not self.capture or not self.capture.isOpened():
            return False, ""
        
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.active_camera}_{timestamp}.avi"
            filepath = os.path.join(self.recording_dir, filename)
            
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(
                filepath, fourcc, self.fps, (self.width, self.height))
            
            if self.video_writer.isOpened():
                self.recording = True
                self.current_recording_file = filepath
                self.get_logger().info(f'Recording started: {filepath}')
                return True, filepath
            else:
                return False, ""
                
        except Exception as e:
            self.get_logger().error(f'Recording start error: {e}')
            return False, ""
    
    def stop_recording(self):
        """Stop video recording"""
        if not self.recording:
            return False
        
        try:
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
            self.recording = False
            self.get_logger().info(f'Recording stopped: {self.current_recording_file}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Recording stop error: {e}')
            return False
    
    def take_snapshot(self):
        """Capture single frame and save"""
        if not self.capture or not self.capture.isOpened():
            return False, ""
        
        try:
            ret, frame = self.capture.read()
            if not ret:
                return False, ""
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.active_camera}_snapshot_{timestamp}.jpg"
            filepath = os.path.join(self.recording_dir, filename)
            
            cv2.imwrite(filepath, frame)
            self.get_logger().info(f'Snapshot saved: {filepath}')
            return True, filepath
            
        except Exception as e:
            self.get_logger().error(f'Snapshot error: {e}')
            return False, ""
    
    def capture_and_publish(self):
        """Capture frame and publish to ROS topics"""
        if not self.capture or not self.capture.isOpened():
            return
        
        try:
            ret, frame = self.capture.read()
            if not ret:
                return
            
            # Write to recording if active
            if self.recording and self.video_writer:
                self.video_writer.write(frame)
            
            # Publish raw image
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = f'camera_{self.active_camera}_optical'
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'Image publish error: {e}', throttle_duration_sec=5.0)
            
            # Publish compressed
            try:
                _, buffer = cv2.imencode('.jpg', frame)
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = self.get_clock().now().to_msg()
                compressed_msg.header.frame_id = f'camera_{self.active_camera}_optical'
                compressed_msg.format = 'jpeg'
                compressed_msg.data = buffer.tobytes()
                self.compressed_pub.publish(compressed_msg)
            except Exception as e:
                pass  # Compressed is optional
                
        except Exception as e:
            self.get_logger().error(f'Capture error: {e}', throttle_duration_sec=5.0)
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.recording:
            self.stop_recording()
        if self.capture:
            self.capture.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraManager()
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

echo "  ✓ robot_sensing created"
