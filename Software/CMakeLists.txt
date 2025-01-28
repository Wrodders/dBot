# Set the minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Define the project name
project(dBotSW)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories for common and local headers
include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/common)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(ZMQ REQUIRED libzmq)
include_directories(${ZMQ_INCLUDE_DIRS})
link_directories(${ZMQ_LIBRARY_DIRS})

# Locate cppzmq (header-only library)
find_path(CPPZMQ_INCLUDE_DIR zmq.hpp)
if (NOT CPPZMQ_INCLUDE_DIR)
    message(FATAL_ERROR "cppzmq not found. Please provide the path to cppzmq's zmq.hpp.")
endif()
include_directories(${CPPZMQ_INCLUDE_DIR})


find_package(Boost REQUIRED)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
else()
    message(FATAL_ERROR "Boost not found. Please specify Boost_INCLUDE_DIR and Boost_LIBRARY_DIRS.")
endif()
# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}  # OpenCV headers
    ${ZMQ_INCLUDE_DIRS}     # ZeroMQ headers
    ${Boost_INCLUDE_DIRS}   # Boost headers
    ../inc/                  # Custom headers (if any, adjust the path as needed)
)

add_executable(main src/main.cpp)
add_executable(zmqproxy tools/zmqProxy.cpp)
add_executable(twsbDriver tools/twsbDriver.cpp)
add_executable(opencv_rtsp tools/opencv_rtsp.cpp)
add_executable(zmqSubTest Test/zmqSubTest.cpp)
add_executable(camFilt tools/camFilt.cpp)


# Link both executables to the necessary libraries
target_link_libraries(main PRIVATE 
    ${OpenCV_LIBS}       # Link OpenCV libraries
    ${ZMQ_LIBRARIES}     # Link ZeroMQ libraries
)

target_link_libraries(zmqproxy PRIVATE 
    ${OpenCV_LIBS}       # Link OpenCV libraries
    ${ZMQ_LIBRARIES}     # Link ZeroMQ libraries
    zmq
)

target_link_libraries(twsbDriver PRIVATE 
    ${OpenCV_LIBS}       # Link OpenCV libraries
    ${ZMQ_LIBRARIES}     # Link ZeroMQ libraries
    zmq
    ${Boost_LIBRARIES}
)

target_link_libraries(opencv_rtsp PRIVATE 
    ${OpenCV_LIBS}       # Link OpenCV libraries
    ${ZMQ_LIBRARIES}     # Link ZeroMQ libraries
    zmq
    ${Boost_LIBRARIES}
)

target_link_libraries(zmqSubTest PRIVATE 
    ${OpenCV_LIBS}       # Link OpenCV libraries
    ${ZMQ_LIBRARIES}     # Link ZeroMQ libraries
    zmq
    ${Boost_LIBRARIES}
)

target_link_libraries(camFilt PRIVATE 
    ${OpenCV_LIBS}       # Link OpenCV libraries
    ${ZMQ_LIBRARIES}     # Link ZeroMQ libraries
    zmq
    ${Boost_LIBRARIES}
)

# Include paths for cppzmq and ZeroMQ headers
target_include_directories(main PRIVATE
    ${ZMQ_INCLUDE_DIRS}   # Include ZeroMQ headers (from find_package)
)

target_include_directories(zmqproxy PRIVATE
    ${ZMQ_INCLUDE_DIRS}   # Include ZeroMQ headers (from find_package)
)

target_include_directories(twsbDriver PRIVATE
    ${ZMQ_INCLUDE_DIRS}   # Include ZeroMQ headers (from find_package)
)

target_include_directories(opencv_rtsp PRIVATE
    ${ZMQ_INCLUDE_DIRS}   # Include ZeroMQ headers (from find_package)
)
target_include_directories(zmqSubTest PRIVATE
    ${ZMQ_INCLUDE_DIRS}   # Include ZeroMQ headers (from find_package)
)

target_include_directories(camFilt PRIVATE
    ${ZMQ_INCLUDE_DIRS}   # Include ZeroMQ headers (from find_package)
)


