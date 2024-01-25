# CMake version
cmake_minimum_required(VERSION 3.12)

# Project name
project(dbot)
# Set the path to the Homebrew installation
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")

# Set the output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#Find Libraries
find_library(SERIALPORT_LIBRARY NAMES serialport)

if(NOT SERIALPORT_LIBRARY)
    message(FATAL_ERROR "libserialport not found")
endif()




# Add your source files
set(SOURCES
    src/main.c
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})


target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

# Optionally, you can organize your source files into folders
# source_group("src" FILES ${SOURCES})

# Optionally, you can add include directories
target_include_directories(${PROJECT_NAME} PRIVATE lib)

#Link Libraries
target_include_directories(${PROJECT_NAME}  PRIVATE "/opt/homebrew/include")
target_link_libraries(${PROJECT_NAME}  PRIVATE "${SERIALPORT_LIBRARY}")

