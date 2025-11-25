# BinBuddy Testing Guide

## Quick Start Testing

### 1. Launch System
```bash
ros2 launch robot_bringup bringup.launch.py
```

### 2. Enable Movement
```bash
ros2 topic pub /movement/enable std_msgs/Bool "data: true" --once
```

### 3. Test Movement
```bash
# Forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --rate 10

# Stop with Ctrl+C, then turn
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.3}}" --rate 10
```

### 4. Test Crane
```bash
# Home
ros2 service call /crane/command robot_interfaces/srv/CraneCommand "{command: 'HOME'}"

# Pick position
ros2 service call /crane/command robot_interfaces/srv/CraneCommand "{command: 'PICK'}"
```

## Troubleshooting

### Arduino Not Found
```bash
# Find Arduino port
ls /dev/tty{ACM,USB}* 2>/dev/null

# Check permissions
groups $USER  # Should include 'dialout'
```

### Movement Not Working
```bash
# Check if enabled
ros2 topic echo /system/health

# Emergency stop if needed
ros2 topic pub /estop std_msgs/Bool "data: true" --once
```

## Common Issues

1. **Arduino at /dev/ttyACM1**: Update launch file or use auto-detection
2. **NumPy error**: `pip3 install "numpy<2" --break-system-packages`
3. **Permission denied**: `sudo usermod -a -G dialout $USER` then logout/login
