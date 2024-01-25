cmake_minimum_required(VERSION 3.10)
project(serialComs C)


set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Add source files
set(SRC_FILES
    serialComs.c
    lib/stringlib.c
)

# Add header files
set(LIB_FILES
    lib/stringlib.h
)

# Create an executable
add_executable(serialComs ${SRC_FILES})

# Include the inc/ folder for header files
target_include_directories(serialComs PRIVATE lib)

# Create a library
add_library(stringlib STATIC lib/stringlib.c)

# Link the library to the executable
target_link_libraries(serialComs PRIVATE stringlib)
