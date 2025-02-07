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
echo "[UPDATE_SW][INFO] Using software directory: $SOFTWARE_DIR"

# Ensure remote directory exists
ssh "$USER@$HOSTNAME" "mkdir -p prog/software/inc"

# Sync project files, excluding unnecessary files
echo "[UPDATE_SW][INFO] Syncing software files to $USER@$HOSTNAME..."
rsync -az --exclude 'mediamtx/' "$SOFTWARE_DIR/" "$USER@$HOSTNAME:prog/software"



# Build the project remotely
echo "[UPDATE_SW][INFO] Running remote build script..."
ssh "$USER@$HOSTNAME" << EOF
    cd prog/software
    bash buildSW.sh
EOF

echo "[UPDATE_SW][SUCCESS] Software build completed successfully on $HOSTNAME!"
