#!/bin/bash
###############################################################################
# BinBuddy Smart Update Script
# Only updates what has changed since last deployment
###############################################################################

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$HOME/binbuddy_ws"
BACKUP_DIR="$HOME/.binbuddy_backups/$(date +%Y%m%d_%H%M%S)"

# Source helpers
source "$REPO_DIR/scripts/version_manager.sh" >/dev/null 2>&1 || true

echo -e "${BLUE}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║       BinBuddy Smart Update - Delta Deployment          ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if initial deployment exists
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${YELLOW}No existing deployment found.${NC}"
    echo -e "${YELLOW}Please run ./deploy.sh for initial deployment.${NC}"
    exit 1
fi

# Get versions
DEPLOYED_VERSION=$(bash "$REPO_DIR/scripts/version_manager.sh" get 2>/dev/null || echo "0.0.0")
REPO_VERSION=$(cat "$REPO_DIR/VERSION" 2>/dev/null || echo "1.0.1")

echo -e "${BLUE}Current version:${NC} $DEPLOYED_VERSION"
echo -e "${BLUE}Available version:${NC} $REPO_VERSION"
echo ""

# Check what changed
echo -e "${GREEN}[1/5] Analyzing changes...${NC}"
CHANGED_FILES=$(bash "$REPO_DIR/scripts/checksum_manager.sh" files 2>/dev/null || echo "")

if [ -z "$CHANGED_FILES" ]; then
    echo -e "${GREEN}✓ No changes detected. System is up to date.${NC}"
    exit 0
fi

echo -e "${YELLOW}Changed files:${NC}"
echo "$CHANGED_FILES" | head -10
if [ $(echo "$CHANGED_FILES" | wc -l) -gt 10 ]; then
    echo "... and $(($(echo "$CHANGED_FILES" | wc -l) - 10)) more"
fi
echo ""

# Categorize changes
BOOTSTRAP_CHANGED=false
SCRIPTS_CHANGED=false
LAUNCH_CHANGED=false
CONFIG_CHANGED=false

echo "$CHANGED_FILES" | while read file; do
    case "$file" in
        */bootstrap/*) echo "bootstrap" >> /tmp/binbuddy_update_flags ;;
        */scripts/*) echo "scripts" >> /tmp/binbuddy_update_flags ;;
        *launch.py) echo "launch" >> /tmp/binbuddy_update_flags ;;
        */config/*) echo "config" >> /tmp/binbuddy_update_flags ;;
    esac
done

if [ -f /tmp/binbuddy_update_flags ]; then
    grep -q "bootstrap" /tmp/binbuddy_update_flags && BOOTSTRAP_CHANGED=true
    grep -q "scripts" /tmp/binbuddy_update_flags && SCRIPTS_CHANGED=true
    grep -q "launch" /tmp/binbuddy_update_flags && LAUNCH_CHANGED=true
    grep -q "config" /tmp/binbuddy_update_flags && CONFIG_CHANGED=true
    rm -f /tmp/binbuddy_update_flags
fi

echo -e "${GREEN}[2/5] Creating backup...${NC}"
mkdir -p "$BACKUP_DIR"
if [ -d "$WORKSPACE_DIR/src/robot_bringup" ]; then
    cp -r "$WORKSPACE_DIR/src/robot_bringup" "$BACKUP_DIR/" 2>/dev/null || true
fi
if [ -d "$WORKSPACE_DIR/src/robot_base" ]; then
    cp -r "$WORKSPACE_DIR/src/robot_base" "$BACKUP_DIR/" 2>/dev/null || true
fi
echo -e "${GREEN}✓ Backup saved to: $BACKUP_DIR${NC}"

echo -e "${GREEN}[3/5] Applying updates...${NC}"

# Update bootstrap scripts if changed
if [ "$BOOTSTRAP_CHANGED" = true ]; then
    echo -e "${YELLOW}Updating bootstrap scripts...${NC}"
    
    # Only regenerate changed packages
    cd "$WORKSPACE_DIR/src"
    
    if echo "$CHANGED_FILES" | grep -q "bootstrap_bringup"; then
        echo "  Regenerating robot_bringup..."
        bash "$REPO_DIR/bootstrap/bootstrap_bringup.sh" "$WORKSPACE_DIR/src"
    fi
    
    if echo "$CHANGED_FILES" | grep -q "bootstrap_base_crane"; then
        echo "  Regenerating robot_base and crane_control..."
        bash "$REPO_DIR/bootstrap/bootstrap_base_crane.sh" "$WORKSPACE_DIR/src"
    fi
    
    # Add more as needed...
fi

# Update scripts if changed
if [ "$SCRIPTS_CHANGED" = true ]; then
    echo -e "${YELLOW}Updating scripts...${NC}"
    cp -r "$REPO_DIR/scripts/"* "$WORKSPACE_DIR/../binbuddy/scripts/" 2>/dev/null || true
fi

# Update launch files if changed
if [ "$LAUNCH_CHANGED" = true ]; then
    echo -e "${YELLOW}Updating launch files...${NC}"
    
    # Copy changed launch files
    echo "$CHANGED_FILES" | grep "launch.py" | while read launch_file; do
        # Determine package
        pkg=$(echo "$launch_file" | grep -oP 'robot_\w+' | head -1)
        if [ -n "$pkg" ] && [ -d "$WORKSPACE_DIR/src/$pkg" ]; then
            echo "  Updating $pkg launch files..."
            cp "$REPO_DIR/$launch_file" "$WORKSPACE_DIR/src/$pkg/launch/" 2>/dev/null || true
        fi
    done
fi

# Update config files if changed
if [ "$CONFIG_CHANGED" = true ]; then
    echo -e "${YELLOW}Updating configuration files...${NC}"
    if [ -d "$WORKSPACE_DIR/src/robot_nlp" ]; then
        # Don't overwrite .env with actual keys
        if [ -f "$REPO_DIR/config/.env.example" ]; then
            if [ ! -f "$WORKSPACE_DIR/src/robot_nlp/.env" ]; then
                cp "$REPO_DIR/config/.env.example" "$WORKSPACE_DIR/src/robot_nlp/.env"
            fi
        fi
    fi
fi

echo -e "${GREEN}[4/5] Rebuilding affected packages...${NC}"

cd "$WORKSPACE_DIR"
source /opt/ros/jazzy/setup.bash

# Determine which packages to rebuild
PACKAGES_TO_BUILD=""

if [ "$BOOTSTRAP_CHANGED" = true ] || [ "$LAUNCH_CHANGED" = true ]; then
    # Rebuild all if major changes
    echo "  Full rebuild required..."
    colcon build --symlink-install --parallel-workers 2
else
    # Selective rebuild
    if echo "$CHANGED_FILES" | grep -q "robot_bringup"; then
        PACKAGES_TO_BUILD="$PACKAGES_TO_BUILD robot_bringup"
    fi
    if echo "$CHANGED_FILES" | grep -q "robot_base"; then
        PACKAGES_TO_BUILD="$PACKAGES_TO_BUILD robot_base"
    fi
    
    if [ -n "$PACKAGES_TO_BUILD" ]; then
        echo "  Rebuilding: $PACKAGES_TO_BUILD"
        colcon build --symlink-install --packages-select $PACKAGES_TO_BUILD
    else
        echo "  No rebuild needed"
    fi
fi

echo -e "${GREEN}[5/5] Updating version tracking...${NC}"
bash "$REPO_DIR/scripts/version_manager.sh" set "$REPO_VERSION"
bash "$REPO_DIR/scripts/checksum_manager.sh" save

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║              Update Complete!                            ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Updated from:${NC} $DEPLOYED_VERSION → $REPO_VERSION"
echo -e "${BLUE}Backup location:${NC} $BACKUP_DIR"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "1. Test the system:"
echo "   ros2 launch robot_bringup bringup.launch.py"
echo ""
echo "2. If issues occur, rollback with:"
echo "   bash $REPO_DIR/scripts/rollback.sh $BACKUP_DIR"
echo ""
