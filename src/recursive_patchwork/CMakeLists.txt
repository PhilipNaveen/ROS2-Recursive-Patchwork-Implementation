cmake_minimum_required(VERSION 3.16)

# Enable languages first
project(recursive_patchwork)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ament_cmake
find_package(ament_cmake REQUIRED)

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

# Find ROS2 packages
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_definitions(-DUSE_ROS2)

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
    src/rosbag_loader.cpp
    src/recursive_patchwork_node.cpp
    src/main.cpp
)

# Create main executable
add_executable(recursive_patchwork ${SOURCES})

# Create ROS2 node executable
add_executable(recursive_patchwork_node src/recursive_patchwork_node.cpp)

# Link libraries for main executable
target_link_libraries(recursive_patchwork
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
)

if(CUDA_FOUND)
    target_link_libraries(recursive_patchwork ${CUDA_LIBRARIES})
endif()

# Link libraries for ROS2 node executable
target_link_libraries(recursive_patchwork_node
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

if(CUDA_FOUND)
    target_link_libraries(recursive_patchwork_node ${CUDA_LIBRARIES})
endif()

# Add ROS2 dependencies using ament_target_dependencies
ament_target_dependencies(recursive_patchwork
    rclcpp
    sensor_msgs
    rosbag2_cpp
)

ament_target_dependencies(recursive_patchwork_node
    rclcpp
    sensor_msgs
    rosbag2_cpp
    visualization_msgs
    pcl_conversions
)

# Compiler flags for main executable
target_compile_options(recursive_patchwork PRIVATE
    -O3
    -march=native
    -ffast-math
    -Wall
    -Wextra
)

# Compiler flags for ROS2 node executable
target_compile_options(recursive_patchwork_node PRIVATE
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

# Add ROS2 libraries to test
ament_target_dependencies(test_recursive_patchwork
    rclcpp
    sensor_msgs
    rosbag2_cpp
)

# Compiler flags for test executable
target_compile_options(test_recursive_patchwork PRIVATE
    -O2
    -Wall
    -Wextra
    -g
)

# Install targets
install(TARGETS 
    recursive_patchwork 
    recursive_patchwork_node
    test_recursive_patchwork
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.hpp"
)

# Install launch files
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(recursive_patchwork recursive_patchwork_node)
ament_export_dependencies(
    rclcpp
    sensor_msgs
    rosbag2_cpp
    visualization_msgs
    pcl_conversions
    Eigen3
    OpenCV
)

# Enable testing
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    
    # Add test
    add_test(NAME test_recursive_patchwork
             COMMAND test_recursive_patchwork)
endif()

# Print configuration summary
message(STATUS "=== Recursive Patchwork Configuration ===")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "CUDA support: ${CUDA_FOUND}")
message(STATUS "Eigen3 version: ${EIGEN3_VERSION}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "ROS2 support: Enabled")
message(STATUS "ROS2: ${rclcpp_VERSION}")
message(STATUS "==========================================")

ament_package() 