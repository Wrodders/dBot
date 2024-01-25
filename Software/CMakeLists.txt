# CMake version
cmake_minimum_required(VERSION 3.12)

# Project name
project(dbot)

# Set the output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Add your source files
set(SOURCES
    src/main.c
    lib/stringlib.c
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})


target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

# Optionally, you can organize your source files into folders
# source_group("src" FILES ${SOURCES})

# Optionally, you can add include directories
target_include_directories(${PROJECT_NAME} PRIVATE inc lib)


