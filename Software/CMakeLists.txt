# Set the minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Define the project name
project(dBotSW)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(fmt REQUIRED)

# Find ZeroMQ using pkg-config
pkg_check_modules(ZMQ REQUIRED libzmq)

# Find cppzmq (header-only library)
find_path(CPPZMQ_INCLUDE_DIR zmq.hpp)
if (NOT CPPZMQ_INCLUDE_DIR)
    message(FATAL_ERROR "cppzmq not found. Please install cppzmq and ensure zmq.hpp is available.")
endif()

# Define common include directories and libraries
set(COMMON_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/common
    ${OpenCV_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${fmt_INCLUDE_DIRS}
    ${CPPZMQ_INCLUDE_DIR}
)

set(COMMON_LIBRARIES
    ${OpenCV_LIBS}
    ${ZMQ_LIBRARIES}
    ${Boost_LIBRARIES}
    fmt::fmt  # Use the correct fmt target
    zmq
)

# Add executables
add_executable(main src/main.cpp)
add_executable(zmqproxy tools/zmqProxy.cpp)
add_executable(zmqComs tools/zmqComs.cpp)
add_executable(opencv_rtsp tools/opencv_rtsp.cpp)
add_executable(zmqSubTest Test/zmqSubTest.cpp)

# Apply common properties to all targets
foreach(target main zmqproxy zmqComs opencv_rtsp zmqSubTest)
    target_include_directories(${target} PRIVATE ${COMMON_INCLUDE_DIRS})
    target_link_libraries(${target} PRIVATE ${COMMON_LIBRARIES})
endforeach()