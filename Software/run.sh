#!/bin/bash

# Function to map names to executable paths
get_executable_path() {
    case "$1" in
        "piComs") echo "./pi/build/bin/piComs" ;;
        "opencvTest") echo "./pc/build/bin/opencvTest" ;;
        "serialReader") echo "./pc/build/bin/serialReader";;
        *) echo "" ;;
    esac
}

# Check if the user has provided at least one argument
if [ $# -lt 1 ]; then
    echo "Usage: $0 <name> [args...]"
    exit 1
fi

# Get the name from the command line argument
name="$1"

# Get the executable path from the mapping function
executable=$(get_executable_path "$name")

# Check if the executable path is empty (name not found)
if [ -z "$executable" ]; then
    echo "Error: $name is not mapped to any executable."
    exit 1
fi

# Shift the first argument (name) out of the argument list
shift

# Check if the executable exists
if [ ! -x "$executable" ]; then
    echo "Error: $executable does not exist or is not executable."
    exit 1
fi

# Run the executable with the remaining arguments
echo "Running $executable ..."
"$executable" "$@"
