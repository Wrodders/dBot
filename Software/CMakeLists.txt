# Minimum CMake version
cmake_minimum_required(VERSION 3.10)

# Project name
project(VideoROI)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Manually set paths for ZeroMQ (since pkg-config may not work with Homebrew path)
set(ZMQ_INCLUDE_DIRS "/opt/homebrew/Cellar/zeromq/4.3.5_1/include")
set(ZMQ_LIBRARIES "/opt/homebrew/Cellar/zeromq/4.3.5_1/lib/libzmq.dylib")

# Check the paths for debugging purposes
message(STATUS "ZeroMQ include dirs: ${ZMQ_INCLUDE_DIRS}")
message(STATUS "ZeroMQ libraries: ${ZMQ_LIBRARIES}")

# Include directories for OpenCV, ZeroMQ, and the source directory
include_directories(
    ${OpenCV_INCLUDE_DIRS}  # OpenCV include directories
    ${ZMQ_INCLUDE_DIRS}     # ZeroMQ include directories
    ${CMAKE_SOURCE_DIR}/include  # Your source include directory (if necessary)
)

# Add the executable
add_executable(VideoROI src/main.cpp)

# Link libraries for OpenCV and ZeroMQ
target_link_libraries(VideoROI PRIVATE
    ${OpenCV_LIBS}   # Link OpenCV libraries
    ${ZMQ_LIBRARIES} # Link ZeroMQ libraries
)

# Ensure the executable is built with the correct include directories
target_include_directories(VideoROI PRIVATE
    ${OpenCV_INCLUDE_DIRS}  # OpenCV include directories
    ${ZMQ_INCLUDE_DIRS}     # ZeroMQ include directories
)
