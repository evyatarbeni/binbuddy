#!/bin/bash
###############################################################################
# BinBuddy Main Deployment Script
# Entry point: git clone → ./deploy.sh → fully operational robot
###############################################################################

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$HOME/binbuddy_ws"

clear
cat << "EOFBANNER"
╔══════════════════════════════════════════════════════════════════╗
║   ██████╗ ██╗███╗   ██╗██████╗ ██╗   ██╗██████╗ ██████╗ ██╗   ║
║   ██╔══██╗██║████╗  ██║██╔══██╗██║   ██║██╔══██╗██╔══██╗╚██╗  ║
║   ██████╔╝██║██╔██╗ ██║██████╔╝██║   ██║██║  ██║██║  ██║ ╚██╗ ║
║   ██╔══██╗██║██║╚██╗██║██╔══██╗██║   ██║██║  ██║██║  ██║ ██╔╝ ║
║   ██████╔╝██║██║ ╚████║██████╔╝╚██████╔╝██████╔╝██████╔╝██╔╝  ║
║   ╚═════╝ ╚═╝╚═╝  ╚═══╝╚═════╝  ╚═════╝ ╚═════╝ ╚═════╝ ╚═╝   ║
║              BinBuddy Deployment - Evyatar Beni                  ║
╚══════════════════════════════════════════════════════════════════╝
EOFBANNER

echo -e "${BLUE}Starting deployment...${NC}\n"

echo -e "${GREEN}[1/10] Pre-flight checks...${NC}"
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "24.04" ]; then
        echo -e "${RED}ERROR: Ubuntu 24.04 required${NC}"
        exit 1
    fi
fi

echo -e "\n${GREEN}[2/10] Installing dependencies...${NC}"
bash "$REPO_DIR/scripts/install_dependencies.sh"

echo -e "\n${GREEN}[3/10] Setting up cameras...${NC}"
bash "$REPO_DIR/scripts/setup_cameras.sh"

echo -e "\n${GREEN}[4/10] Generating ROS 2 packages...${NC}"
bash "$REPO_DIR/bootstrap/bootstrap.sh"

echo -e "\n${GREEN}[5/10] Installing LiDAR driver...${NC}"
cd "$WORKSPACE_DIR/src"
if [ ! -d "ldlidar_stl_ros2" ]; then
    git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
fi

echo -e "\n${GREEN}[6/10] Building workspace (10-15 minutes)...${NC}"
cd "$WORKSPACE_DIR"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 2

echo -e "\n${GREEN}[7/10] Configuring environment...${NC}"
if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# BinBuddy ROS 2 Workspace" >> ~/.bashrc
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
fi

echo -e "\n${GREEN}[8/10] Setting up devices...${NC}"
bash "$REPO_DIR/scripts/setup_udev_rules.sh"
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER

echo -e "\n${GREEN}[9/10] Final setup...${NC}"
mkdir -p "$HOME/maps" "$HOME/binbuddy_recordings"

# Copy .env if not exists
if [ ! -f "$WORKSPACE_DIR/src/robot_nlp/.env" ]; then
    cp "$REPO_DIR/config/.env.example" "$WORKSPACE_DIR/src/robot_nlp/.env"
fi

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                  DEPLOYMENT SUCCESSFUL!                          ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}NEXT STEPS:${NC}"
echo "  1. nano $WORKSPACE_DIR/src/robot_nlp/.env  # Add OpenAI key"
echo "  2. sudo reboot"
echo "  3. ros2 launch robot_bringup bringup.launch.py mode:=teleop"
echo ""
read -p "Reboot now? (y/n) " -n 1 -r
echo
[[ $REPLY =~ ^[Yy]$ ]] && sudo reboot
