cmake_minimum_required(VERSION 3.10)
project(dBot)

# Set the path to the Homebrew installation
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")

set(CMAKE_C_STANDARD 11)

# Set the output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Find Libraries
find_library(SERIALPORT_LIBRARY NAMES serialport)
find_library(ZMQ_LIBRARY NAMES zmq PATHS /opt/homebrew/lib)

# Add source files
set(SOURCES
    src/piComs.c
)

# Create the executable
add_executable(piComs ${SOURCES})
target_compile_options(piComs PRIVATE -Wall -Wextra)
target_include_directories(piComs PRIVATE lib "/opt/homebrew/include")
target_link_libraries(piComs PRIVATE "${SERIALPORT_LIBRARY}" "${ZMQ_LIBRARY}")
set_target_properties(piComs PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
