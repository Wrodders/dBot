#!/bin/bash

# Set the name of your C++ program
program_name= "opencvTest"

# Set the path to your C++ source file
source_file="opencvTest.cpp"

# Set the path to the OpenCV installation
opencv_path="opt/homebrew/Cellar/opencv/4.9.0_1/"

# Compilation command
g++ -o "$program_name" "$source_file" -L"$opencv_path"/lib -I"$opencv_path"/include -lopencv_core -lopencv_highgui -lopencv_imgcodecs

# Check if the compilation was successful
if [ $? -eq 0 ]; then
    echo "Compilation successful. Running the program..."
    ./"$program_name"
else
    echo "Compilation failed."
fi
