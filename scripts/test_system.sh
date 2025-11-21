#!/bin/bash
# Quick system test

set -e

echo "BinBuddy System Test"
echo "===================="

echo ""
echo "1. Checking ROS 2 installation..."
if command -v ros2 &> /dev/null; then
    echo "✓ ROS 2 found: $(ros2 --version)"
else
    echo "✗ ROS 2 not found"
fi

echo ""
echo "2. Checking workspace..."
if [ -d "$HOME/binbuddy_ws/install" ]; then
    echo "✓ Workspace built"
    source "$HOME/binbuddy_ws/install/setup.bash"
    echo "✓ Workspace sourced"
else
    echo "✗ Workspace not built"
fi

echo ""
echo "3. Checking devices..."

if [ -e /dev/ttyACM0 ] || [ -e /dev/arduino_mega ]; then
    echo "✓ Arduino connected"
else
    echo "✗ Arduino not found"
fi

if [ -e /dev/ttyUSB0 ] || [ -e /dev/ldrobot_lidar ]; then
    echo "✓ LiDAR connected"
else
    echo "✗ LiDAR not found"
fi

if ls /dev/video* 1> /dev/null 2>&1; then
    echo "✓ Camera devices found:"
    ls /dev/video*
else
    echo "✗ No camera devices"
fi

echo ""
echo "4. Testing Pi Cameras..."
if command -v libcamera-hello &> /dev/null; then
    echo "✓ libcamera installed"
    libcamera-hello --list-cameras
else
    echo "✗ libcamera not installed"
fi

echo ""
echo "5. Checking ROS 2 packages..."
ros2 pkg list | grep robot | head -10

echo ""
echo "Test complete!"
