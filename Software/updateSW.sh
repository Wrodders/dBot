#!/bin/bash

# Exit on any error
set -e

# Help function
function show_help {
    echo "Usage: $0 [-h] [-u USER] [-i HOSTNAME] [-p DEVICE]"
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
echo "[INFO] Using software directory: $SOFTWARE_DIR"

# Ensure remote directory exists
ssh "$USER@$HOSTNAME" "mkdir -p prog/software/inc"

# Sync project files, excluding unnecessary files
echo "[INFO] Syncing software files to $USER@$HOSTNAME..."
rsync -az --exclude 'mediamtx/mediamtx' --exclude 'mediamtx/mediamtx_v1.11.2_darwin_amd64.tar.gz' "$SOFTWARE_DIR/" "$USER@$HOSTNAME:prog/software"

# Sync mcuComs.h file separately
echo "[INFO] Syncing MCU headers..."
rsync -az ../Firmware/pubrpc/mcuComs.h "$USER@$HOSTNAME:prog/software/inc"

# Build the project remotely
echo "[INFO] Running remote build script..."
ssh "$USER@$HOSTNAME" << EOF
    cd prog/software
    bash buildSW.sh
EOF

echo "[SUCCESS] Software build completed successfully on $HOSTNAME!"
