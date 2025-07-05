cmake_minimum_required(VERSION 3.16)
project(recursive_patchwork)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# Optional: CUDA support
find_package(CUDA QUIET)
if(CUDA_FOUND)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    add_definitions(-DUSE_CUDA)
    message(STATUS "CUDA acceleration enabled")
else()
    message(STATUS "CUDA acceleration not available")
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

# Source files
set(SOURCES
    src/recursive_patchwork.cpp
    src/point_cloud_processor.cpp
    src/lidar_fusion.cpp
    src/rosbag_loader.cpp
    src/visualization.cpp
    src/main.cpp
)

# Create executable
add_executable(recursive_patchwork ${SOURCES})

# Link libraries
target_link_libraries(recursive_patchwork
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    rosbag2_cpp::rosbag2_cpp
)

if(CUDA_FOUND)
    target_link_libraries(recursive_patchwork ${CUDA_LIBRARIES})
endif()

# Compiler flags
target_compile_options(recursive_patchwork PRIVATE
    -O3
    -march=native
    -ffast-math
    -Wall
    -Wextra
)

# Install
install(TARGETS recursive_patchwork
    RUNTIME DESTINATION bin
) 