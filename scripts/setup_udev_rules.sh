#!/bin/bash
# Setup udev rules for Arduino and LiDAR

set -e

echo "Creating udev rules..."

sudo tee /etc/udev/rules.d/99-binbuddy.rules > /dev/null <<'EOF'
# Arduino Mega
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", MODE="0666", SYMLINK+="arduino_mega", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0010", MODE="0666", SYMLINK+="arduino_mega", GROUP="dialout"

# LDROBOT STL-19P LiDAR (CP2102 USB-UART)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="ldrobot_lidar", GROUP="dialout"

# USB Camera
SUBSYSTEM=="video4linux", ATTRS{name}=="*USB*", MODE="0666", GROUP="video"
EOF

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "✓ Udev rules configured"
