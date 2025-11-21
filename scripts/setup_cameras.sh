#!/bin/bash
# Setup camera support for USB + 2x Pi Cameras

set -e

echo "Installing camera libraries..."
sudo apt install -y \
    libcamera-tools \
    libcamera-dev \
    gstreamer1.0-libcamera \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    v4l-utils

echo "Configuring camera in boot config..."
if [ -f /boot/firmware/config.txt ]; then
    if ! grep -q "camera_auto_detect=1" /boot/firmware/config.txt; then
        echo "camera_auto_detect=1" | sudo tee -a /boot/firmware/config.txt
        echo "dtoverlay=imx708" | sudo tee -a /boot/firmware/config.txt
    fi
fi

echo "✓ Camera support configured"
echo "Note: Reboot required for camera changes to take effect"
