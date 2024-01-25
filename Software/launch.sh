#!/bin/bash

# Check if an argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <executable> [build|run|clean]"
    exit 1
fi

# Set the executable name
EXECUTABLE=$1

# Set the build directory
BUILD_DIR=build
BINARY_DIR="$BUILD_DIR/bin"


# Function to build the project
build_project() {
    if [ ! -d "$BUILD_DIR" ]; then
        mkdir "$BUILD_DIR"
    fi

    cd "$BUILD_DIR"
    cmake ..
    make
    cd ..
}

# Function to run the executable
run_executable() {
    if [ -x "$BINARY_DIR/$EXECUTABLE" ]; then
        shift 2
        "$BINARY_DIR/$EXECUTABLE" "$@"
    else
        echo "Executable '$EXECUTABLE' not found. Please build the project first."
    fi
}

# Function to clean the project
clean_project() {
    if [ -d "$BUILD_DIR" ]; then
        rm -rf "$BUILD_DIR"
        echo "Project cleaned."
    else
        echo "Nothing to clean."
    fi
}

# Parse user input
case "$2" in
    "build")
        build_project
        ;;
    "run")
        run_executable "$@"
        ;;
    "clean")
        clean_project
        ;;
    *)
        echo "Invalid option. Usage: $0 <executable> [build|run|clean]"
        exit 1
        ;;
esac
