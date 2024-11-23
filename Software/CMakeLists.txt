cmake_minimum_required(VERSION 3.10)
project(VideoROI)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Include OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} include)

# Add executable
add_executable(VideoROI src/main.cpp src/roi_selector.cpp)

# Link OpenCV libraries
target_link_libraries(VideoROI ${OpenCV_LIBS})
