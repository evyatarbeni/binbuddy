#!/bin/bash
###############################################################################
# BinBuddy Main Deployment Script
# Entry point: git clone → ./deploy.sh → fully operational robot
#
# Author: Evyatar Beni
# Email: evyatarbeni@gmail.com
# Version: 1.0.1
###############################################################################

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$HOME/binbuddy_ws"
REPO_VERSION="1.0.1"

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

echo -e "${BLUE}Starting BinBuddy deployment v${REPO_VERSION}...${NC}\n"

###############################################################################
# STEP 1: Pre-flight Checks
###############################################################################
echo -e "${GREEN}[1/11] Pre-flight checks...${NC}"

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "24.04" ]; then
        echo -e "${RED}ERROR: Ubuntu 24.04 is required${NC}"
        echo -e "${RED}Current version: Ubuntu $VERSION_ID${NC}"
        exit 1
    fi
    echo -e "  ${GREEN}✓${NC} Ubuntu 24.04 detected"
else
    echo -e "${YELLOW}  ⚠ Could not verify Ubuntu version${NC}"
fi

# Check if running on Raspberry Pi
if grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    PI_MODEL=$(grep "Model" /proc/cpuinfo | cut -d: -f2 | xargs)
    echo -e "  ${GREEN}✓${NC} Raspberry Pi detected: $PI_MODEL"
else
    echo -e "${YELLOW}  ⚠ Not running on Raspberry Pi (testing mode)${NC}"
fi

# Check internet connectivity
if ping -c 1 8.8.8.8 >/dev/null 2>&1; then
    echo -e "  ${GREEN}✓${NC} Internet connection available"
else
    echo -e "${RED}  ✗${NC} No internet connection"
    echo -e "${RED}Please connect to the internet and try again${NC}"
    exit 1
fi

# Check available disk space (need at least 5GB)
AVAILABLE_SPACE=$(df -BG "$HOME" | tail -1 | awk '{print $4}' | sed 's/G//')
if [ "$AVAILABLE_SPACE" -lt 5 ]; then
    echo -e "${RED}  ✗${NC} Insufficient disk space (need 5GB, have ${AVAILABLE_SPACE}GB)"
    exit 1
fi
echo -e "  ${GREEN}✓${NC} Sufficient disk space (${AVAILABLE_SPACE}GB available)"

###############################################################################
# STEP 2: Install Dependencies
###############################################################################
echo -e "\n${GREEN}[2/11] Installing dependencies...${NC}"
if [ ! -f "$REPO_DIR/scripts/install_dependencies.sh" ]; then
    echo -e "${RED}ERROR: install_dependencies.sh not found${NC}"
    exit 1
fi
bash "$REPO_DIR/scripts/install_dependencies.sh"

# Verify ROS 2 installation
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo -e "${RED}ERROR: ROS 2 Jazzy installation failed${NC}"
    exit 1
fi
echo -e "  ${GREEN}✓${NC} ROS 2 Jazzy installed successfully"

###############################################################################
# STEP 3: Setup Cameras
###############################################################################
echo -e "\n${GREEN}[3/11] Setting up cameras...${NC}"
if [ -f "$REPO_DIR/scripts/setup_cameras.sh" ]; then
    bash "$REPO_DIR/scripts/setup_cameras.sh"
else
    echo -e "${YELLOW}  ⚠ Camera setup script not found, skipping${NC}"
fi

###############################################################################
# STEP 4: Create Workspace
###############################################################################
echo -e "\n${GREEN}[4/11] Creating ROS 2 workspace...${NC}"

if [ -d "$WORKSPACE_DIR" ]; then
    echo -e "${YELLOW}  ⚠ Workspace already exists at $WORKSPACE_DIR${NC}"
    read -p "  Remove and recreate? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo -e "  Backing up existing workspace..."
        mv "$WORKSPACE_DIR" "${WORKSPACE_DIR}_backup_$(date +%Y%m%d_%H%M%S)"
    else
        echo -e "${YELLOW}  Using existing workspace${NC}"
    fi
fi

mkdir -p "$WORKSPACE_DIR/src"
echo -e "  ${GREEN}✓${NC} Workspace created at $WORKSPACE_DIR"

###############################################################################
# STEP 5: Generate ROS 2 Packages
###############################################################################
echo -e "\n${GREEN}[5/11] Generating ROS 2 packages...${NC}"

if [ ! -f "$REPO_DIR/bootstrap/bootstrap.sh" ]; then
    echo -e "${RED}ERROR: bootstrap.sh not found${NC}"
    exit 1
fi

bash "$REPO_DIR/bootstrap/bootstrap.sh" "$WORKSPACE_DIR"

# Verify packages were created
EXPECTED_PACKAGES="robot_interfaces robot_base crane_control robot_description robot_sensing robot_navigation robot_slam robot_teleop robot_nlp robot_bringup"
for pkg in $EXPECTED_PACKAGES; do
    if [ ! -d "$WORKSPACE_DIR/src/$pkg" ]; then
        echo -e "${RED}ERROR: Package $pkg was not created${NC}"
        exit 1
    fi
done
echo -e "  ${GREEN}✓${NC} All 10 packages generated successfully"

###############################################################################
# STEP 6: Install and Patch LiDAR Driver
###############################################################################
echo -e "\n${GREEN}[6/11] Installing LiDAR driver...${NC}"

cd "$WORKSPACE_DIR/src"

if [ ! -d "ldlidar_stl_ros2" ]; then
    echo -e "  Cloning LiDAR driver..."
    git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
    if [ $? -ne 0 ]; then
        echo -e "${RED}ERROR: Failed to clone LiDAR driver${NC}"
        exit 1
    fi
    echo -e "  ${GREEN}✓${NC} LiDAR driver cloned"
else
    echo -e "${YELLOW}  LiDAR driver already exists${NC}"
fi

# Apply pthread patch for Ubuntu 24.04
echo -e "  Patching LiDAR driver for Ubuntu 24.04..."
LIDAR_LOG_FILE="$WORKSPACE_DIR/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp"

if [ -f "$LIDAR_LOG_FILE" ]; then
    if ! grep -q "#include <pthread.h>" "$LIDAR_LOG_FILE"; then
        sed -i '18a #include <pthread.h>' "$LIDAR_LOG_FILE"
        echo -e "  ${GREEN}✓${NC} LiDAR driver patched"
    else
        echo -e "  ${GREEN}✓${NC} LiDAR driver already patched"
    fi
else
    echo -e "${YELLOW}  ⚠ LiDAR log file not found, skipping patch${NC}"
fi

###############################################################################
# STEP 7: Build Workspace
###############################################################################
echo -e "\n${GREEN}[7/11] Building workspace (this will take 10-15 minutes)...${NC}"

cd "$WORKSPACE_DIR"

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo -e "${RED}ERROR: Cannot source ROS 2 environment${NC}"
    exit 1
fi

# Verify ROS environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}ERROR: ROS_DISTRO not set${NC}"
    exit 1
fi
echo -e "  ROS_DISTRO: $ROS_DISTRO"

# Build with error handling
echo -e "  ${BLUE}Building packages...${NC}"
colcon build --symlink-install --parallel-workers 2 --executor sequential

if [ $? -ne 0 ]; then
    echo -e "${RED}ERROR: Workspace build failed${NC}"
    echo -e "${YELLOW}Check logs at: $WORKSPACE_DIR/log/latest_build${NC}"
    exit 1
fi

# Verify build artifacts
if [ ! -d "$WORKSPACE_DIR/install" ]; then
    echo -e "${RED}ERROR: Build succeeded but install directory not created${NC}"
    exit 1
fi

echo -e "  ${GREEN}✓${NC} Workspace built successfully"

###############################################################################
# STEP 8: Configure Environment
###############################################################################
echo -e "\n${GREEN}[8/11] Configuring environment...${NC}"

# Add to .bashrc if not already present
if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# BinBuddy ROS 2 Workspace - Added by deploy.sh" >> ~/.bashrc
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
    echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
    echo -e "  ${GREEN}✓${NC} Added ROS 2 environment to ~/.bashrc"
else
    echo -e "  ${GREEN}✓${NC} ROS 2 environment already in ~/.bashrc"
fi

# Source for current session
source "$WORKSPACE_DIR/install/setup.bash"

###############################################################################
# STEP 9: Setup Device Permissions
###############################################################################
echo -e "\n${GREEN}[9/11] Setting up device permissions...${NC}"

# Run udev rules setup
if [ -f "$REPO_DIR/scripts/setup_udev_rules.sh" ]; then
    bash "$REPO_DIR/scripts/setup_udev_rules.sh"
else
    echo -e "${YELLOW}  ⚠ udev rules script not found${NC}"
fi

# Add user to required groups
echo -e "  Adding user to dialout and video groups..."
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER

# Check if user is in groups (won't take effect until logout)
if groups $USER | grep -q dialout; then
    echo -e "  ${GREEN}✓${NC} User in dialout group"
else
    echo -e "  ${YELLOW}⚠ User added to dialout (requires logout)${NC}"
fi

if groups $USER | grep -q video; then
    echo -e "  ${GREEN}✓${NC} User in video group"
else
    echo -e "  ${YELLOW}⚠ User added to video (requires logout)${NC}"
fi

###############################################################################
# STEP 10: Create Required Directories
###############################################################################
echo -e "\n${GREEN}[10/11] Creating required directories...${NC}"

mkdir -p "$HOME/maps"
mkdir -p "$HOME/binbuddy_recordings"
mkdir -p "$HOME/.binbuddy_backups"

echo -e "  ${GREEN}✓${NC} Created ~/maps"
echo -e "  ${GREEN}✓${NC} Created ~/binbuddy_recordings"
echo -e "  ${GREEN}✓${NC} Created ~/.binbuddy_backups"

# Copy .env template for NLP
if [ -f "$REPO_DIR/config/.env.example" ]; then
    if [ ! -f "$WORKSPACE_DIR/src/robot_nlp/.env" ]; then
        cp "$REPO_DIR/config/.env.example" "$WORKSPACE_DIR/src/robot_nlp/.env"
        echo -e "  ${GREEN}✓${NC} .env template copied"
    else
        echo -e "  ${GREEN}✓${NC} .env file already exists"
    fi
fi

###############################################################################
# STEP 11: Initialize Version Tracking
###############################################################################
echo -e "\n${GREEN}[11/11] Initializing version tracking...${NC}"

# Create VERSION file if it doesn't exist
if [ ! -f "$REPO_DIR/VERSION" ]; then
    echo "$REPO_VERSION" > "$REPO_DIR/VERSION"
fi

# Initialize version manager
if [ -f "$REPO_DIR/scripts/version_manager.sh" ]; then
    bash "$REPO_DIR/scripts/version_manager.sh" set "$REPO_VERSION"
    echo -e "  ${GREEN}✓${NC} Version set to $REPO_VERSION"
fi

# Initialize checksum tracking
if [ -f "$REPO_DIR/scripts/checksum_manager.sh" ]; then
    bash "$REPO_DIR/scripts/checksum_manager.sh" save >/dev/null 2>&1
    echo -e "  ${GREEN}✓${NC} File checksums saved"
fi

###############################################################################
# DEPLOYMENT COMPLETE
###############################################################################
echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                  DEPLOYMENT SUCCESSFUL!                          ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Installation Summary:${NC}"
echo -e "  Version: $REPO_VERSION"
echo -e "  Workspace: $WORKSPACE_DIR"
echo -e "  Packages: 11 (10 custom + ldlidar)"
echo ""
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}                        NEXT STEPS                                 ${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${RED}1. Add OpenAI API key (optional for NLP features):${NC}"
echo -e "   ${BLUE}nano $WORKSPACE_DIR/src/robot_nlp/.env${NC}"
echo -e "   Replace: ${YELLOW}OPENAI_API_KEY=your-openai-api-key-here${NC}"
echo ""
echo -e "${RED}2. REBOOT (required for group permissions):${NC}"
echo -e "   ${BLUE}sudo reboot${NC}"
echo ""
echo -e "${GREEN}3. After reboot, test the system:${NC}"
echo -e "   ${BLUE}bash $REPO_DIR/scripts/test_system.sh${NC}"
echo ""
echo -e "${GREEN}4. Launch BinBuddy:${NC}"
echo -e "   ${BLUE}ros2 launch robot_bringup bringup.launch.py${NC}"
echo ""
echo -e "${GREEN}5. Enable movement (in another terminal):${NC}"
echo -e "   ${BLUE}ros2 topic pub /movement/enable std_msgs/Bool \"data: true\" --once${NC}"
echo ""
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${BLUE}📚 Documentation:${NC} $REPO_DIR/docs/"
echo -e "${BLUE}🐛 Issues:${NC} https://github.com/evyatarbeni/binbuddy/issues"
echo -e "${BLUE}📧 Contact:${NC} evyatarbeni@gmail.com"
echo ""
echo -e "${YELLOW}⚠️  Important Notes:${NC}"
echo -e "  • Arduino may be at /dev/ttyACM1 (auto-detected)"
echo -e "  • Movement is DISABLED by default (safety feature)"
echo -e "  • LiDAR requires /dev/ttyUSB0 connection"
echo -e "  • Reboot is REQUIRED for group permissions"
echo ""

# Offer to reboot
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}Reboot now to complete installation? (y/n)${NC}"
read -p "> " -n 1 -r
echo
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${GREEN}Rebooting in 5 seconds...${NC}"
    sleep 5
    sudo reboot
else
    echo ""
    echo -e "${YELLOW}Remember to reboot before launching the robot!${NC}"
    echo -e "Run: ${BLUE}sudo reboot${NC}"
    echo ""
fi

exit 0
