#!/bin/bash
# Install all system dependencies for BinBuddy

set -e

echo "Updating system..."
sudo apt update
sudo apt upgrade -y

echo "Installing build tools..."
sudo apt install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    git \
    wget \
    python3-pip \
    build-essential \
    cmake \
    libudev-dev \
    libusb-1.0-0-dev \
    xterm

echo "Installing ROS 2 Jazzy..."
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
fi

sudo apt install -y \
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
    ros-jazzy-rviz2 \
    python3-colcon-common-extensions

echo "Installing Python dependencies..."
pip3 install --break-system-packages \
    openai \
    python-dotenv \
    pyserial \
    opencv-python \
    numpy

echo "✓ Dependencies installed successfully"
