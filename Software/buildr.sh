#!/bin/bash

# Set project variables
BUILD_DIR="build"
EXECUTABLE_NAME="VideoROI"

# Create the build directory if it doesn't exist
if [ ! -d "$BUILD_DIR" ]; then
    echo "Creating build directory..."
    mkdir "$BUILD_DIR"
fi

# Navigate to the build directory
cd "$BUILD_DIR"

# Run CMake and build the project
echo "Configuring project with CMake..."
cmake ..
if [ $? -ne 0 ]; then
    echo "CMake configuration failed. Exiting."
    exit 1
fi

echo "Building the project..."
make
if [ $? -ne 0 ]; then
    echo "Build failed. Exiting."
    exit 1
fi

# Navigate back to the project root
cd ..

# Run the executable with default parameters
echo "Running the program..."
./"$BUILD_DIR"/"$EXECUTABLE_NAME" --source camera
