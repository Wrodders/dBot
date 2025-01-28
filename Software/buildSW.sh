#!/bin/bash

# Enable strict error handling
set -euo pipefail

# Function to display an error message and exit
function error_exit {
    echo "Error: $1"
    exit 1
}

# Remove existing build directory
rm -rf build || error_exit "Failed to remove 'build' directory."

# Create a new build directory
mkdir build || error_exit "Failed to create 'build' directory."

# Navigate into the build directory
cd build || error_exit "Failed to enter 'build' directory."

# Run CMake
cmake .. || error_exit "CMake configuration failed."

# Build the project
make twsbDriver || error_exit "Build process failed."

make zmqproxy || error_exit "Build process failed."

make zmqSubTest || error_exit "Build process failed."

make opencv_rtsp || error_exit "Build process failed."

make camFilt || error_exit "Build process failed."


echo "Build completed successfully!"
