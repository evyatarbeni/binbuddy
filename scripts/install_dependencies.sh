#!/bin/bash
###############################################################################
# Install all system dependencies for BinBuddy
# Assumes ROS 2 repository is already configured
###############################################################################

set -e

echo "Installing system packages..."

# Check if ROS 2 repo exists, if not add it
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    echo "ROS 2 repository not found, adding it..."
    
    sudo apt install -y curl gnupg lsb-release
    
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

# Update package lists
sudo apt update
sudo apt upgrade -y

###############################################################################
# Install ROS 2 and Dependencies
###############################################################################
echo "Installing ROS 2 Jazzy and dependencies..."

sudo apt install -y \
    git wget build-essential cmake libudev-dev libusb-1.0-0-dev xterm \
    python3-pip python3-colcon-common-extensions \
    ros-jazzy-desktop \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-camera-info-manager \
    ros-jazzy-v4l2-camera \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-joy-linux \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2

###############################################################################
# Python Dependencies
###############################################################################
echo "Installing Python packages..."

pip3 install --break-system-packages \
    openai \
    python-dotenv \
    pyserial \
    opencv-python \
    numpy

###############################################################################
# Verification
###############################################################################
echo ""
echo "✓ All dependencies installed successfully"
echo ""
