# BinBuddy Quick Start Guide

## Initial Setup (One Time)

### 1. Prepare Raspberry Pi 5

1. Flash Ubuntu 24.04 Server to SD card: https://ubuntu.com/download/raspberry-pi
2. Boot and complete initial setup
3. Connect to internet (WiFi or Ethernet)
4. Update system: `sudo apt update && sudo apt upgrade -y`

### 2. Deploy BinBuddy
```bash
# Clone repository
cd ~
git clone https://github.com/evyatarbeni/binbuddy.git
cd binbuddy

# Run deployment (takes 30-45 minutes)
chmod +x deploy.sh
./deploy.sh
```

### 3. Configure

1. Add OpenAI API key:
```bash
   nano ~/binbuddy_ws/src/robot_nlp/.env
   # Add your key, save with Ctrl+X
```

2. Reboot:
```bash
   sudo reboot
```

## Daily Operation

### Start Robot (Teleop Mode)
```bash
# Launch everything
ros2 launch robot_bringup bringup.launch.py mode:=teleop use_rviz:=true

# In another terminal, enable movement
ros2 topic pub /movement/enable std_msgs/Bool "data: true" --once
```

### Keyboard Controls

- **W/↑**: Forward
- **S/↓**: Backward  
- **A/←**: Turn left
- **D/→**: Turn right
- **X/Space**: Stop
- **1**: Crane stow position
- **2**: Crane pick position
- **3**: Crane place position
- **E**: Enable movement
- **Q**: Disable movement (safe mode)
- **ESC**: Emergency stop & exit

### Camera Controls
```bash
# Switch to USB camera
ros2 topic pub /camera/switch std_msgs/String "data: 'usb'" --once

# Switch to Pi Camera 1 (front)
ros2 topic pub /camera/switch std_msgs/String "data: 'picam1'" --once

# Switch to Pi Camera 2 (rear)
ros2 topic pub /camera/switch std_msgs/String "data: 'picam2'" --once

# Take snapshot
ros2 service call /camera/control robot_interfaces/srv/CameraCommand \
  "{command: 'snapshot', camera_name: 'usb'}"

# Start recording
ros2 service call /camera/control robot_interfaces/srv/CameraCommand \
  "{command: 'start_recording', camera_name: 'picam1'}"

# Stop recording
ros2 service call /camera/control robot_interfaces/srv/CameraCommand \
  "{command: 'stop_recording', camera_name: 'picam1'}"
```

## Building a Map
```bash
# Launch with SLAM
ros2 launch robot_bringup bringup.launch.py mode:=teleop use_slam:=true use_rviz:=true

# Drive around your space
# When done, save map:
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/office
```

## Autonomous Navigation
```bash
# Launch with your map
ros2 launch robot_bringup bringup.launch.py \
  mode:=autonomy \
  map:=~/maps/office.yaml \
  use_rviz:=true

# In RViz:
# 1. Click "2D Pose Estimate" and set robot's initial position
# 2. Click "Nav2 Goal" and set destination
```

## Natural Language Commands
```bash
# Send text command
ros2 service call /nlp/command robot_interfaces/srv/NLPCommand \
  "{command_text: 'go to bin A and pick it up'}"

# Full delivery sequence
ros2 service call /nlp/command robot_interfaces/srv/NLPCommand \
  "{command_text: 'pick up bin from bin_a, deliver to drop_zone_b, then return to dock'}"
```

## Monitoring
```bash
# Check system health
ros2 topic echo /system/health

# Monitor LiDAR
ros2 topic hz /scan

# Monitor cameras
ros2 topic hz /camera/usb/image_raw
ros2 topic hz /camera/picam1/image_raw

# Check odometry
ros2 topic echo /odom

# View all topics
ros2 topic list

# Check TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

## Troubleshooting

See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for common issues and solutions.
