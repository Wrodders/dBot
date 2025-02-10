#!/bin/bash
# Enable strict error handling
set -euo pipefail

# Function to display an error message and exit
function error_exit {
    echo "Error: $1"
    exit 1
}

echo "[INFO] Building the project..."
# Build the project
make clean || error_exit "Clean failed."
make all -j4 || error_exit "Build failed." 

echo "[BUILD_SW][INFO]Build completed successfully!"
