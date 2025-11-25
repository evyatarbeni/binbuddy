#!/bin/bash
###############################################################################
# BinBuddy Rollback Script
# Restore previous version from backup
###############################################################################

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

BACKUP_DIR="${1:-}"
WORKSPACE_DIR="$HOME/binbuddy_ws"

if [ -z "$BACKUP_DIR" ]; then
    echo -e "${RED}Error: Backup directory not specified${NC}"
    echo "Usage: $0 <backup_directory>"
    echo ""
    echo "Available backups:"
    ls -lt "$HOME/.binbuddy_backups/" 2>/dev/null | head -10
    exit 1
fi

if [ ! -d "$BACKUP_DIR" ]; then
    echo -e "${RED}Error: Backup directory not found: $BACKUP_DIR${NC}"
    exit 1
fi

echo -e "${YELLOW}WARNING: This will restore from backup:${NC}"
echo "  $BACKUP_DIR"
echo ""
read -p "Continue? (yes/no): " CONFIRM

if [ "$CONFIRM" != "yes" ]; then
    echo "Rollback cancelled"
    exit 0
fi

echo -e "${GREEN}Rolling back...${NC}"

# Restore packages
if [ -d "$BACKUP_DIR/robot_bringup" ]; then
    rm -rf "$WORKSPACE_DIR/src/robot_bringup"
    cp -r "$BACKUP_DIR/robot_bringup" "$WORKSPACE_DIR/src/"
    echo "✓ Restored robot_bringup"
fi

if [ -d "$BACKUP_DIR/robot_base" ]; then
    rm -rf "$WORKSPACE_DIR/src/robot_base"
    cp -r "$BACKUP_DIR/robot_base" "$WORKSPACE_DIR/src/"
    echo "✓ Restored robot_base"
fi

# Rebuild
cd "$WORKSPACE_DIR"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 2

echo -e "${GREEN}✓ Rollback complete${NC}"
