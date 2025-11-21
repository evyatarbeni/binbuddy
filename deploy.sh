#!/bin/bash
###############################################################################
# BinBuddy One-Command Deployment Script
# For Raspberry Pi 5 running Ubuntu 24.04
#
# Author: Evyatar Beni
# Email: evyatarbeni@gmail.com
# Repository: https://github.com/evyatarbeni/binbuddy
#
# Usage: ./deploy.sh
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

REPO_DIR="$HOME/binbuddy"
WORKSPACE_DIR="$HOME/binbuddy_ws"

###############################################################################
# Banner
###############################################################################
clear
cat << "EOF"
╔══════════════════════════════════════════════════════════════════╗
║                                                                  ║
║   ██████╗ ██╗███╗   ██╗██████╗ ██╗   ██╗██████╗ ██████╗ ██╗   ║
║   ██╔══██╗██║████╗  ██║██╔══██╗██║   ██║██╔══██╗██╔══██╗╚██╗  ║
║   ██████╔╝██║██╔██╗ ██║██████╔╝██║   ██║██║  ██║██║  ██║ ╚██╗ ║
║   ██╔══██╗██║██║╚██╗██║██╔══██╗██║   ██║██║  ██║██║  ██║ ██╔╝ ║
║   ██████╔╝██║██║ ╚████║██████╔╝╚██████╔╝██████╔╝██████╔╝██╔╝  ║
║   ╚═════╝ ╚═╝╚═╝  ╚═══╝╚═════╝  ╚═════╝ ╚═════╝ ╚═════╝ ╚═╝   ║
║                                                                  ║
║         Autonomous Bin Delivery Robot - Deployment              ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝

EOF

echo -e "${BLUE}Starting deployment from GitHub repository...${NC}\n"
sleep 2

###############################################################################
# Pre-flight checks
###############################################################################
echo -e "${GREEN}[1/10] Running pre-flight checks...${NC}"

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "24.04" ]; then
        echo -e "${RED}ERROR: Ubuntu 24.04 required. Current: $VERSION_ID${NC}"
        echo "Please install Ubuntu 24.04 Server for Raspberry Pi"
        exit 1
    fi
    echo -e "${GREEN}✓ Ubuntu 24.04 detected${NC}"
fi

# Check if running on Raspberry Pi
if grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    echo -e "${GREEN}✓ Running on Raspberry Pi${NC}"
else
    echo -e "${YELLOW}⚠ Not detected as Raspberry Pi${NC}"
fi

# Check if repository exists
if [ ! -d "$REPO_DIR" ]; then
    echo -e "${RED}ERROR: Repository not found at $REPO_DIR${NC}"
    echo "Please run from the repository directory"
    exit 1
fi

cd "$REPO_DIR"

###############################################################################
# Install dependencies
###############################################################################
echo -e "\n${GREEN}[2/10] Installing system dependencies...${NC}"
bash scripts/install_dependencies.sh

###############################################################################
# Setup cameras
###############################################################################
echo -e "\n${GREEN}[3/10] Setting up camera support...${NC}"
bash scripts/setup_cameras.sh

###############################################################################
# Create workspace and copy packages
###############################################################################
echo -e "\n${GREEN}[4/10] Creating ROS 2 workspace...${NC}"
mkdir -p "$WORKSPACE_DIR/src"

echo -e "${GREEN}[5/10] Copying packages to workspace...${NC}"
cp -r "$REPO_DIR/src"/* "$WORKSPACE_DIR/src/"

###############################################################################
# Install LDROBOT LiDAR driver
###############################################################################
echo -e "\n${GREEN}[6/10] Installing LDROBOT STL-19P LiDAR driver...${NC}"
cd "$WORKSPACE_DIR/src"
if [ ! -d "ldlidar_stl_ros2" ]; then
    git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
fi

###############################################################################
# Build workspace
###############################################################################
echo -e "\n${GREEN}[7/10] Building ROS 2 workspace (this may take 10-15 minutes)...${NC}"
cd "$WORKSPACE_DIR"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

###############################################################################
# Setup environment
###############################################################################
echo -e "\n${GREEN}[8/10] Configuring environment...${NC}"

# Add to bashrc
if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS 2 BinBuddy workspace" >> ~/.bashrc
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
    echo -e "${GREEN}✓ Added workspace to ~/.bashrc${NC}"
fi

# Create .env file from example
if [ ! -f "$WORKSPACE_DIR/src/robot_nlp/.env" ]; then
    if [ -f "$REPO_DIR/config/.env.example" ]; then
        cp "$REPO_DIR/config/.env.example" "$WORKSPACE_DIR/src/robot_nlp/.env"
        echo -e "${YELLOW}⚠ Created .env file - you must add your OpenAI API key${NC}"
    fi
fi

###############################################################################
# Setup udev rules
###############################################################################
echo -e "\n${GREEN}[9/10] Setting up device permissions...${NC}"
bash "$REPO_DIR/scripts/setup_udev_rules.sh"

###############################################################################
# Final setup
###############################################################################
echo -e "\n${GREEN}[10/10] Finalizing installation...${NC}"

# Add user to groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER

# Create maps directory
mkdir -p "$HOME/maps"

# Copy documentation to home
cp -r "$REPO_DIR/docs" "$HOME/binbuddy_docs"

###############################################################################
# Success message
###############################################################################
echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                    DEPLOYMENT SUCCESSFUL!                        ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}IMPORTANT NEXT STEPS:${NC}"
echo ""
echo "1. Add your OpenAI API key:"
echo "   ${BLUE}nano $WORKSPACE_DIR/src/robot_nlp/.env${NC}"
echo ""
echo "2. ${RED}REBOOT REQUIRED${NC} for permissions to take effect:"
echo "   ${BLUE}sudo reboot${NC}"
echo ""
echo "3. After reboot, test cameras:"
echo "   ${BLUE}libcamera-hello --camera 0  # Pi Camera 1${NC}"
echo "   ${BLUE}libcamera-hello --camera 1  # Pi Camera 2${NC}"
echo ""
echo "4. Launch the robot:"
echo "   ${BLUE}ros2 launch robot_bringup bringup.launch.py mode:=teleop${NC}"
echo ""
echo -e "${GREEN}Documentation:${NC} ~/binbuddy_docs/"
echo -e "${GREEN}Quick test:${NC} bash $REPO_DIR/scripts/test_system.sh"
echo ""
echo -e "${YELLOW}Reboot now? (y/n)${NC}"
read -p "> " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo reboot
fi
