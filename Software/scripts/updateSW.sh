#!/bin/bash
# -------------------------------------------------- 
#  @file    update_sw.cpp
#  @brief   Deploys and builds software on the robot remotely.
#  @date    2025-01-05
#  @version 1.0
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

bash scripts/syncSW.sh -u $USER -i $HOSTNAME

# Build the project remotely
echo "[UPDATE_SW][INFO] Running remote build script..."
ssh "$USER@$HOSTNAME" << EOF
    cd prog/software
    bash scripts/buildSW.sh
EOF

echo "[UPDATE_SW][SUCCESS] Software build completed successfully on $HOSTNAME!"
