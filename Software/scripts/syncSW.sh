#!/bin/bash
# -------------------------------------------------- 
#  @file     syncSW.cpp
#  @brief   Synchronizes software files to the robot remotely.
#  @details Used as part of the build and deploy process.

set -e

# Help function
function show_help {
    echo "Usage: $0 [-h] [-u USER] [-i HOSTNAME]"
    echo
    echo "Options:"
    echo "  -u USER        Specify the username (default: pi)."
    echo "  -i HOSTNAME    Specify the hostname (default: bot.local)."
    echo "  -h             Show this help message and exit."
    echo
    exit 0
}

# Default values
USER="pi"
HOSTNAME="bot.local"

# Parse command-line arguments
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -u|--user) USER="$2"; shift 2;;
        -i|--hostname) HOSTNAME="$2"; shift 2;;
        -h|--help) show_help;;
        *) echo "[ERROR] Unknown parameter: $1"; show_help;;
    esac
done



# Get the current software directory
SOFTWARE_DIR=$(pwd)
echo "[UPDATE_SW][INFO] Using software directory: $SOFTWARE_DIR"

# Ensure remote directory exists
ssh "$USER@$HOSTNAME" "mkdir -p prog/software/inc"

# Sync project files, excluding unnecessary files
echo "[UPDATE_SW][INFO] Syncing software files to $USER@$HOSTNAME..."
rsync -az --exclude 'mediamtx/'  --exclude 'build/' "$SOFTWARE_DIR/" "$USER@$HOSTNAME:prog/software"