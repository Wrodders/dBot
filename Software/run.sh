#!/bin/bash

# Check if at least one argument is provided
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <executable_name> [arguments...]"
  exit 1
fi

# Extract the executable name from the arguments
executable="$1"
shift

# Change to the build directory
cd build

# Run the executable with the remaining arguments
./"$executable" "$@"

# Return to the original directory
cd ..
