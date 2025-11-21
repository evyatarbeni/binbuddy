# 🤖 BinBuddy - Autonomous Bin Delivery Robot

ROS 2 Jazzy-based autonomous robot for bin pickup and delivery using differential drive, crane, LiDAR navigation, and natural language control.

## Hardware Requirements

- Raspberry Pi 5 (4GB+ RAM)
- Ubuntu 24.04 Server for Raspberry Pi
- Arduino Mega 2560
- LDROBOT STL-19P LiDAR
- 1x USB Camera
- 2x Raspberry Pi Cameras (IMX708 or similar)
- Differential drive base (4 DC motors)
- Crane/hoist actuator with limit switches
- Motor drivers

## Quick Start (Fresh Raspberry Pi 5 with Ubuntu 24.04)

### One-Command Deployment
```bash
git clone https://github.com/evyatarbeni/binbuddy.git
cd binbuddy
./deploy.sh
