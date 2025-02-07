# Set the minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Define the project name
project(dBotSW)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(fmt REQUIRED)
find_package(cppzmq REQUIRED)
find_package(nlohmann_json REQUIRED)

# Find ZeroMQ using pkg-config
pkg_check_modules(ZMQ REQUIRED libzmq)
# Define common include directories and libraries
set(COMMON_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/common
    ${OpenCV_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${fmt_INCLUDE_DIRS}
    ${CPPZMQ_INCLUDE_DIR}
    ${nlohmann_json_INCLUDE_DIRS} 
)

set(COMMON_LIBRARIES
    ${OpenCV_LIBS}
    ${ZMQ_LIBRARIES}
    ${Boost_LIBRARIES}
    fmt::fmt
    zmq
    nlohmann_json::nlohmann_json
)

# Add executables
add_executable(main src/main.cpp)
add_executable(zmqproxy tools/zmqProxy.cpp)
add_executable(twsbComs tools/twsbComs.cpp)
add_executable(vision tools/vision.cpp)
add_executable(zmqSubTest Test/zmqSubTest.cpp)
add_executable(zmqPubTest Test/zmqPubTest.cpp)

# Apply common properties to all targets
foreach(target main zmqproxy twsbComs vision zmqSubTest zmqPubTest)
    target_include_directories(${target} PRIVATE ${COMMON_INCLUDE_DIRS})
    target_link_libraries(${target} PRIVATE ${COMMON_LIBRARIES})
endforeach()

# Ensure that the pkg-config library paths are linked correctly
link_directories(${ZMQ_LIBRARY_DIRS})
