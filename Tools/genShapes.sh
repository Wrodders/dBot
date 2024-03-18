#!/bin/bash

# Define default values
clean=false

# Parse command line options
while getopts ":c" opt; do
  case $opt in
    c)
      clean=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

# Ensure the output directory exists
mkdir -p output_images

# Clean the output directory if requested
if [ "$clean" = true ]; then
    rm -rf output_images/*
    echo "Output directory cleaned."
fi

# Define shape types
shapes=("hexagon" "circle" "rectangle")

# Loop over shape types
for shape in "${shapes[@]}"; do
    # Counter for sequential numbers
    counter=1

    # Loop over random side lengths
    for _ in {1..3}; do
        side_length=$((RANDOM % 50 + 30))  # Random side length between 50 and 80
        # Loop over random spacings
        for _ in {1..3}; do
            min_spacing=$((side_length + 20))
            spacing=$((RANDOM % 91 + min_spacing))  # Random spacing between min_spacing and min_spacing + 90

            # Generate images with vertical orientation
            output_name="${shape}${counter}.bmp"
            python bmpGen.py \
                172 \
                320 \
                "$shape" \
                "$side_length" \
                "$spacing" \
                --orientation "vertical" \
                --output "output_images/$output_name"

            # Increment the counter
            ((counter++))
        done
    done
done
