#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <c_file>"
  exit 1
fi

c_file="$1"
output_file="${c_file%.c}"

# Compile the C file
gcc -o "$output_file" "$c_file"

# Check if compilation was successful
if [ $? -eq 0 ]; then
  # Run the compiled executable
  "./$output_file"
else
  echo "Compilation failed."
fi
