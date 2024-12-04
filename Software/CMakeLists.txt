cmake_minimum_required(VERSION 3.10)

project(VideoROI)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find ZeroMQ



find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
include_directories(${ZMQ_INCLUDE_DIRS})
link_directories(${ZMQ_LIBRARY_DIRS})

find_package(ZeroMQ REQUIRED)

# Locate cppzmq (header-only library)
find_path(CPPZMQ_INCLUDE_DIR zmq.hpp)
if (NOT CPPZMQ_INCLUDE_DIR)
    message(FATAL_ERROR "cppzmq not found. Please provide the path to cppzmq's zmq.hpp.")
endif()
include_directories(${CPPZMQ_INCLUDE_DIR})

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}  # OpenCV headers
    ../inc/                  # Custom headers (if any, adjust the path as needed)
)

# Source files
set(SOURCE_FILES
    src/main.cpp            # Add more source files if needed
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Link libraries
target_link_libraries(${PROJECT_NAME} 
    PRIVATE 
    ${OpenCV_LIBS}  # OpenCV libraries
    zmq             # ZeroMQ library
)
