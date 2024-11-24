cmake_minimum_required(VERSION 3.10)

# Define project
project(VideoROI)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include directories (correctly adding OpenCV and local include directory)
include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)

# Add the executable
add_executable(VideoROI src/main.cpp)

# Link OpenCV libraries
target_link_libraries(VideoROI PRIVATE ${OpenCV_LIBS})

# Optional: If you are working with specific OpenCV modules, you can link them like this:
# target_link_libraries(VideoROI PRIVATE OpenCV::Core OpenCV::Imgproc OpenCV::VideoCapture)
