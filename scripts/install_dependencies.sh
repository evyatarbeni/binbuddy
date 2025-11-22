#!/bin/bash
set -e

echo "Installing system packages..."

# Clean repository state
sudo killall apt apt-get 2>/dev/null || true
sleep 1
sudo rm -f /etc/apt/sources.list.d/*ros*.list 2>/dev/null || true
sudo rm -f /usr/share/keyrings/ros*.gpg 2>/dev/null || true
sudo sed -i '/packages.ros.org/d' /etc/apt/sources.list 2>/dev/null || true
sudo rm -rf /var/lib/apt/lists/*
sudo mkdir -p /var/lib/apt/lists/partial

# Update
sudo apt update
sudo apt upgrade -y

# Install prerequisites
sudo apt install -y curl gnupg lsb-release

# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# Install ROS 2 with ALL build tools
echo "Installing ROS 2 and build tools..."
sudo apt install -y \
    git wget build-essential cmake libudev-dev libusb-1.0-0-dev xterm \
    python3-pip python3-colcon-common-extensions \
    ros-jazzy-desktop \
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-cmake-core \
    ros-jazzy-ament-cmake-python \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
    ros-jazzy-image-transport ros-jazzy-image-transport-plugins \
    ros-jazzy-camera-info-manager ros-jazzy-v4l2-camera \
    ros-jazzy-cv-bridge ros-jazzy-vision-opencv \
    ros-jazzy-teleop-twist-keyboard ros-jazzy-teleop-twist-joy ros-jazzy-joy-linux \
    ros-jazzy-xacro ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher ros-jazzy-rviz2

# Python packages
echo "Installing Python packages..."
pip3 install --break-system-packages openai python-dotenv pyserial opencv-python numpy

echo "✓ All dependencies installed successfully"

# Verify ROS 2 installation
echo ""
echo "Verifying ROS 2 installation..."
source /opt/ros/jazzy/setup.bash
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS 2 not properly installed"
    exit 1
fi
echo "✓ ROS_DISTRO: $ROS_DISTRO"
echo "✓ Dependencies verified"
