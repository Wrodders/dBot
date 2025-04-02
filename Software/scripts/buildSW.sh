#!/bin/bash
# -------------------------------------------------- 
#  @file    buildSW.cpp
#  @brief   Builds the software project.
#  @date    2025-01-05
#  @version 1.0
set -euo pipefail

# Function to display an error message and exit
function error_exit {
    echo "Error: $1"
    exit 1
}

HOSTNAME=$(hostname)
echo "[INFO] Hostname: $HOSTNAME"

echo "[INFO] Building the project..."
make clean || error_exit "Clean failed."
#if hsot name is dbot build all_ex_vision 
if [ "$HOSTNAME" == "dbot" ]; then
    make all_no_vision -j4 || error_exit "Build failed."
else

make all -j4 || error_exit "Build failed." 
fi

echo "[BUILD_SW][INFO]Build completed successfully!"
