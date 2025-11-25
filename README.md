# 🤖 BinBuddy - Autonomous Bin Delivery Robot

ROS 2 Jazzy-based autonomous robot for bin pickup and delivery using differential drive, crane, LiDAR navigation, and natural language control.

## Hardware Requirements

- Raspberry Pi 5 (4GB+ RAM)
- Ubuntu 24.04 Server
- Arduino Mega 2560
- LDROBOT STL-19P LiDAR
- 1x USB Camera + 2x Raspberry Pi Cameras
- Differential drive base (4 DC motors)
- Crane/hoist actuator with limit switches

## Quick Start

### One-Command Deployment
```bash
cd ~
git clone https://github.com/evyatarbeni/binbuddy.git
cd binbuddy
chmod +x deploy.sh
./deploy.sh
```

**Note**: Arduino may be at `/dev/ttyACM1` instead of `/dev/ttyACM0`. The system will detect this automatically.

### After Deployment

1. **Add OpenAI API key** (optional for NLP):
```bash
   nano ~/binbuddy_ws/src/robot_nlp/.env
```

2. **Reboot**:
```bash
   sudo reboot
```

3. **Launch robot**:
```bash
   ros2 launch robot_bringup bringup.launch.py
```

4. **Enable movement** (in another terminal):
```bash
   ros2 topic pub /movement/enable std_msgs/Bool "data: true" --once
```

## Testing

See [TESTING.md](docs/TESTING.md) for complete testing guide.

Quick test:
```bash
# Enable movement
ros2 topic pub /movement/enable std_msgs/Bool "data: true" --once

# Test forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --rate 10
```

## Known Issues & Fixes

### Arduino Port Detection
If Arduino is at `/dev/ttyACM1`, the system auto-detects this. If manual override needed:
```bash
./scripts/detect_arduino.sh
```

### NumPy Compatibility
If camera fails with NumPy 2.x error:
```bash
pip3 install --break-system-packages "numpy<2"
```

### LiDAR Build Error
Automatically patched during deployment (pthread.h include added).

## Documentation

- [Quick Start Guide](docs/QUICKSTART.md)
- [Hardware Setup](docs/HARDWARE_SETUP.md)
- [Testing Guide](docs/TESTING.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)

## Author

**Evyatar Beni**
Email: evyatarbeni@gmail.com
Repository: https://github.com/evyatarbeni/binbuddy

## License

MIT License
