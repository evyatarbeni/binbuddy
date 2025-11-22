#!/bin/bash
###############################################################################
# BinBuddy Checksum Manager
# Tracks file changes and detects what needs updating
###############################################################################

set -e

CHECKSUM_FILE="$HOME/.binbuddy_checksums"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Generate checksums for all important files
generate_checksums() {
    cd "$REPO_DIR"
    
    # Files to track
    find . -type f \( \
        -name "*.sh" -o \
        -name "*.py" -o \
        -name "*.md" -o \
        -name "*.yaml" -o \
        -name "*.xml" -o \
        -name "*.launch.py" \
    \) ! -path "./.git/*" ! -path "./build/*" | \
    while read file; do
        md5sum "$file"
    done | sort -k2
}

# Save current checksums
save_checksums() {
    generate_checksums > "$CHECKSUM_FILE"
    echo "Checksums saved to $CHECKSUM_FILE"
}

# Compare checksums and report changes
check_changes() {
    if [ ! -f "$CHECKSUM_FILE" ]; then
        echo "No baseline checksums found. Run 'save' first."
        return 1
    fi
    
    local TEMP_FILE=$(mktemp)
    generate_checksums > "$TEMP_FILE"
    
    echo "=== Changed Files ==="
    diff "$CHECKSUM_FILE" "$TEMP_FILE" | grep "^[<>]" | while read line; do
        local status="${line:0:1}"
        local file=$(echo "$line" | awk '{print $2}')
        
        if [ "$status" = "<" ]; then
            echo "MODIFIED or DELETED: $file"
        else
            echo "NEW or MODIFIED: $file"
        fi
    done
    
    rm -f "$TEMP_FILE"
}

# Get list of changed files only
get_changed_files() {
    if [ ! -f "$CHECKSUM_FILE" ]; then
        # No baseline, return all files
        generate_checksums | awk '{print $2}'
        return
    fi
    
    local TEMP_FILE=$(mktemp)
    generate_checksums > "$TEMP_FILE"
    
    diff "$CHECKSUM_FILE" "$TEMP_FILE" | grep "^>" | awk '{print $3}'
    
    rm -f "$TEMP_FILE"
}

# Main
case "${1:-check}" in
    save)
        save_checksums
        ;;
    
    check)
        check_changes
        ;;
    
    files)
        get_changed_files
        ;;
    
    *)
        echo "Usage: $0 {save|check|files}"
        echo "  save  - Save current checksums as baseline"
        echo "  check - Show what files have changed"
        echo "  files - List changed files only"
        exit 1
        ;;
esac
