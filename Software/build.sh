#!/bin/bash

# Function to build PC project
build_pc() {
    echo "Building PC project..."
    cd pc || exit 1
    mkdir -p build
    cd build || exit 1
    cmake ..
    make
    cd ../..
}

# Function to build PI project
build_pi() {
    echo "Building PI project..."
    cd PI || exit 1
    mkdir -p build
    cd build || exit 1
    cmake ..
    make
    cd ../..
}

# Function to build both projects
build_all() {
    build_pc
    build_pi
}

# Main script
case "$1" in
    "pc")
        build_pc
        ;;
    "pi")
        build_pi
        ;;
    "ALL")
        build_all
        ;;
    *)
        echo "Usage: $0 [PC|PI|ALL]"
        exit 1
        ;;
esac
