#!/bin/bash
###############################################################################
# BinBuddy Version Manager
# Tracks deployed version and manages incremental updates
###############################################################################

set -e

VERSION_FILE="$HOME/.binbuddy_version"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Get current deployed version
get_deployed_version() {
    if [ -f "$VERSION_FILE" ]; then
        cat "$VERSION_FILE"
    else
        echo "0.0.0"
    fi
}

# Get repository version
get_repo_version() {
    if [ -f "$REPO_DIR/VERSION" ]; then
        cat "$REPO_DIR/VERSION"
    else
        echo "1.0.1"
    fi
}

# Set deployed version
set_deployed_version() {
    echo "$1" > "$VERSION_FILE"
    echo "timestamp=$(date -Iseconds)" >> "$VERSION_FILE"
    echo "workspace=$HOME/binbuddy_ws" >> "$VERSION_FILE"
}

# Compare versions
version_gt() {
    test "$(printf '%s\n' "$@" | sort -V | head -n 1)" != "$1"
}

# Main
case "${1:-check}" in
    check)
        DEPLOYED=$(get_deployed_version)
        REPO=$(get_repo_version)
        echo "Deployed: $DEPLOYED"
        echo "Available: $REPO"
        if version_gt "$REPO" "$DEPLOYED"; then
            echo "Update available!"
            exit 1
        else
            echo "Up to date"
            exit 0
        fi
        ;;
    
    get)
        get_deployed_version
        ;;
    
    set)
        set_deployed_version "${2:-1.0.1}"
        echo "Version set to: ${2:-1.0.1}"
        ;;
    
    *)
        echo "Usage: $0 {check|get|set <version>}"
        exit 1
        ;;
esac
