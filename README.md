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
# On your development machine (not the Pi)
cd /path/to/your/local/binbuddy
git init
git add .
git commit -m "Initial commit - complete BinBuddy system"
git branch -M main
git remote add origin https://github.com/evyatarbeni/binbuddy.git
git push -u origin main

# On the Raspberry Pi with fresh Ubuntu 24.04
cd ~
git clone https://github.com/evyatarbeni/binbuddy.git
cd binbuddy
chmod +x deploy.sh
./deploy.sh
