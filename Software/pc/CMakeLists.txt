cmake_minimum_required(VERSION 3.10)
project(dBot)

# Set the path to the Homebrew installation
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")

set(CMAKE_CXX_STANDARD 14)

# Set the output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Find Libraries
set(OpenCV_DIR ~/opencv/install/lib/cmake/opencv4)
find_library(SERIALPORT_LIBRARY NAMES serialport)
find_package(OpenCV REQUIRED)

# Add source files
set(SOURCES
    src/main.c
)

# Create the executable

# serialReader
add_executable(serialReader src/serialReader.c)
target_compile_options(serialReader PRIVATE -Wall -Wextra)

target_include_directories(serialReader PRIVATE lib)
target_include_directories(serialReader PRIVATE "/opt/homebrew/include")
target_link_libraries(serialReader PRIVATE "${SERIALPORT_LIBRARY}")
set_target_properties(serialReader PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# openCVTest
add_executable(opencvTest src/opencvTest.cpp)
target_compile_options(opencvTest PRIVATE -Wall -Wextra)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(opencvTest ${OpenCV_LIBS})
target_compile_options(opencvTest PRIVATE -Wall -Wextra)
target_link_libraries(opencvTest ${OpenCV_LIBS})
set_target_properties(opencvTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
