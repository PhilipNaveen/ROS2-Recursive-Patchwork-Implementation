cmake_minimum_required(VERSION 3.16)
project(recursive_patchwork)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

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

# Optional: ROS2 support (only if available)
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(rosbag2_cpp QUIET)

if(rclcpp_FOUND AND sensor_msgs_FOUND AND rosbag2_cpp_FOUND)
    message(STATUS "ROS2 support enabled")
    add_definitions(-DUSE_ROS2)
else()
    message(STATUS "ROS2 support not available - building standalone version")
    add_definitions(-DSTANDALONE_BUILD)
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
    src/visualization.cpp
)

# Add ROS2-specific sources only if ROS2 is available
if(rclcpp_FOUND AND sensor_msgs_FOUND AND rosbag2_cpp_FOUND)
    list(APPEND SOURCES src/rosbag_loader.cpp)
endif()

list(APPEND SOURCES src/main.cpp)

# Create main executable
add_executable(recursive_patchwork ${SOURCES})

# Link libraries for main executable
target_link_libraries(recursive_patchwork
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
)

if(CUDA_FOUND)
    target_link_libraries(recursive_patchwork ${CUDA_LIBRARIES})
endif()

# Add ROS2 libraries if available
if(rclcpp_FOUND AND sensor_msgs_FOUND AND rosbag2_cpp_FOUND)
    target_link_libraries(recursive_patchwork
        ${rclcpp_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${rosbag2_cpp_LIBRARIES}
    )
    target_include_directories(recursive_patchwork PRIVATE
        ${rclcpp_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
    )
endif()

# Compiler flags for main executable
target_compile_options(recursive_patchwork PRIVATE
    -O3
    -march=native
    -ffast-math
    -Wall
    -Wextra
)

# Test executable
set(TEST_SOURCES
    test/test_recursive_patchwork.cpp
    src/recursive_patchwork.cpp
    src/point_cloud_processor.cpp
    src/lidar_fusion.cpp
    src/visualization.cpp
)

add_executable(test_recursive_patchwork ${TEST_SOURCES})

# Link libraries for test executable
target_link_libraries(test_recursive_patchwork
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
)

if(CUDA_FOUND)
    target_link_libraries(test_recursive_patchwork ${CUDA_LIBRARIES})
endif()

# Add ROS2 libraries to test if available
if(rclcpp_FOUND AND sensor_msgs_FOUND AND rosbag2_cpp_FOUND)
    target_link_libraries(test_recursive_patchwork
        ${rclcpp_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${rosbag2_cpp_LIBRARIES}
    )
    target_include_directories(test_recursive_patchwork PRIVATE
        ${rclcpp_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
    )
endif()

# Compiler flags for test executable
target_compile_options(test_recursive_patchwork PRIVATE
    -O2
    -Wall
    -Wextra
    -g
)

# Install targets
install(TARGETS recursive_patchwork test_recursive_patchwork
    RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.hpp"
)

# Install launch files and configs if they exist
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

# Print configuration summary
message(STATUS "=== Recursive Patchwork Configuration ===")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "CUDA support: ${CUDA_FOUND}")
message(STATUS "Eigen3 version: ${EIGEN3_VERSION}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
if(rclcpp_FOUND AND sensor_msgs_FOUND AND rosbag2_cpp_FOUND)
    message(STATUS "ROS2 support: Enabled")
    message(STATUS "ROS2: ${rclcpp_VERSION}")
else()
    message(STATUS "ROS2 support: Disabled (standalone build)")
endif()
message(STATUS "==========================================") 