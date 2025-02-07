#!/bin/bash

# ==============================
# Detect Architecture & Set Paths
# ==============================

ARCH=$(uname -m)
OS=$(uname)

if [[ "$OS" == "Darwin" ]]; then
    echo "Building for macOS (M1/Intel)"
    INCLUDE_DIRS="-I/opt/homebrew/include -I/usr/local/include"
    LIBRARY_DIRS="-L/opt/homebrew/lib -L/usr/local/lib"
    CXXFLAGS="-std=c++17 -Wall -Wextra"
elif [[ "$ARCH" == "aarch64" ]]; then
    echo "Building for Raspberry Pi 4 (64-bit)"
    INCLUDE_DIRS="-I/usr/include -I/usr/local/include"
    LIBRARY_DIRS="-L/usr/lib -L/usr/local/lib"
    CXXFLAGS="-std=c++17 -Wall -Wextra -march=armv8-a"
elif [[ "$ARCH" == "armv7l" ]]; then
    echo "Building for Raspberry Pi 3 (32-bit ARMv7)"
    INCLUDE_DIRS="-I/usr/include -I/usr/local/include"
    LIBRARY_DIRS="-L/usr/lib -L/usr/local/lib"
    CXXFLAGS="-std=c++17 -Wall -Wextra -march=armv7-a"
elif [[ "$ARCH" == "armv6l" ]]; then
    echo "Building for Raspberry Pi Zero (32-bit ARMv6)"
    INCLUDE_DIRS="-I/usr/include -I/usr/local/include"
    LIBRARY_DIRS="-L/usr/lib -L/usr/local/lib"
    CXXFLAGS="-std=c++17 -Wall -Wextra -mcpu=arm1176jzf-s"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

# ==============================
# Define Libraries to Link
# ==============================

LIBRARIES="-lzmq -lfmt -lopencv_core -lboost_system"

# ==============================
# Define Executables & Source Files
# ==============================

declare -A EXECUTABLES
EXECUTABLES=(
    ["main"]="src/main.cpp"
    ["zmqproxy"]="tools/zmqProxy.cpp"
    ["twsbComs"]="tools/twsbComs.cpp"
    ["opencv_rtsp"]="tools/opencv_rtsp.cpp"
    ["zmqSubTest"]="Test/zmqSubTest.cpp"
    ["zmqPubTest"]="Test/zmqPubTest.cpp"
)

# ==============================
# Build Each Executable
# ==============================

for EXE in "${!EXECUTABLES[@]}"; do
    SRC_FILE="${EXECUTABLES[$EXE]}"
    echo "Compiling $EXE from $SRC_FILE..."
    g++ $CXXFLAGS $SRC_FILE -o $EXE $INCLUDE_DIRS $LIBRARY_DIRS $LIBRARIES
    if [[ $? -ne 0 ]]; then
        echo "Compilation failed for $EXE"
        exit 1
    fi
done

echo "Build completed successfully!"
