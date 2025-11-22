#!/bin/bash
###############################################################################
# BinBuddy Master Bootstrap Orchestrator
# Calls individual bootstrap scripts in correct order
###############################################################################

set -e

# Accept workspace path as argument, default to ~/binbuddy_ws
WORKSPACE_DIR="${1:-$HOME/binbuddy_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║     BinBuddy Bootstrap - Generating ROS 2 Workspace     ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Workspace: $WORKSPACE_DIR${NC}"
echo ""

# Create workspace structure
mkdir -p "$WORKSPACE_DIR/src"
cd "$WORKSPACE_DIR/src"

# Execute bootstrap scripts in order
echo -e "${BLUE}Starting package generation...${NC}\n"

bash "$SCRIPT_DIR/bootstrap_interfaces.sh" "$WORKSPACE_DIR/src"
bash "$SCRIPT_DIR/bootstrap_base_crane.sh" "$WORKSPACE_DIR/src"
bash "$SCRIPT_DIR/bootstrap_description.sh" "$WORKSPACE_DIR/src"
bash "$SCRIPT_DIR/bootstrap_sensing.sh" "$WORKSPACE_DIR/src"
bash "$SCRIPT_DIR/bootstrap_navigation_slam.sh" "$WORKSPACE_DIR/src"
bash "$SCRIPT_DIR/bootstrap_teleop_nlp.sh" "$WORKSPACE_DIR/src"
bash "$SCRIPT_DIR/bootstrap_bringup.sh" "$WORKSPACE_DIR/src"

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║      All ROS 2 Packages Generated Successfully!         ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""
echo "Workspace location: $WORKSPACE_DIR"
echo "Ready to build with: cd $WORKSPACE_DIR && colcon build"
