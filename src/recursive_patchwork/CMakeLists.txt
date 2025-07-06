cmake_minimum_required(VERSION 3.16)

# Declare project first
project(recursive_patchwork)

# Find ament_cmake
find_package(ament_cmake REQUIRED)

# Set C++ standard
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
    
    # Set CUDA architectures - use common architectures instead of "native"
    # This avoids GPU detection issues at build time
    set(CMAKE_CUDA_ARCHITECTURES "60;70;75;80;86")
    
    # Add CUDA compiler flags to suppress Eigen warnings
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr")
    
    add_definitions(-DUSE_CUDA)
    
    # Find Thrust (usually comes with CUDA)
    find_path(THRUST_INCLUDE_DIR thrust/device_vector.h
        PATHS
        ${CUDA_INCLUDE_DIRS}
        /usr/local/cuda/include
        /opt/cuda/include
        NO_DEFAULT_PATH
    )
    
    if(THRUST_INCLUDE_DIR)
        include_directories(${THRUST_INCLUDE_DIR})
        message(STATUS "CUDA acceleration enabled with Thrust")
        message(STATUS "CUDA architectures: ${CMAKE_CUDA_ARCHITECTURES}")
        
        # Add CUDA kernel source
        set(CUDA_KERNEL_SOURCES
            cuda/cuda_wrapper.cu
            cuda/cuda_interface.cu
        )
    else()
        message(STATUS "CUDA found but Thrust not available - CUDA acceleration disabled")
        set(CUDA_FOUND FALSE)
    endif()
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

# Source files for the library
set(LIBRARY_SOURCES
    src/recursive_patchwork.cpp
    src/point_cloud_processor.cpp
    src/lidar_fusion.cpp
    src/visualization.cpp
    src/rosbag_loader.cpp
)

# Create library
add_library(recursive_patchwork_lib ${LIBRARY_SOURCES})

# Add CUDA kernels as a separate library if CUDA is available
if(CUDA_FOUND)
    add_library(cuda_kernels_lib ${CUDA_KERNEL_SOURCES})
    set_target_properties(cuda_kernels_lib PROPERTIES
        POSITION_INDEPENDENT_CODE ON
    )
endif()

# Link libraries for the library
target_link_libraries(recursive_patchwork_lib
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
)

if(CUDA_FOUND)
    target_link_libraries(recursive_patchwork_lib 
        ${CUDA_LIBRARIES}
        cuda_kernels_lib
    )
endif()

# Add ROS2 dependencies to the library
ament_target_dependencies(recursive_patchwork_lib
    rclcpp
    sensor_msgs
    rosbag2_cpp
)

# Compiler flags for the library
target_compile_options(recursive_patchwork_lib PRIVATE
    -O3
    -march=native
    -ffast-math
    -Wall
    -Wextra
    -Wno-unused-parameter
)

# Source files for main executable
set(MAIN_SOURCES
    src/main.cpp
)

# Create main executable
add_executable(recursive_patchwork ${MAIN_SOURCES})

# Create ROS2 node executable
add_executable(recursive_patchwork_node src/recursive_patchwork_node.cpp)

# Link libraries for main executable
target_link_libraries(recursive_patchwork
    recursive_patchwork_lib
)

# Link libraries for ROS2 node executable
target_link_libraries(recursive_patchwork_node
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    recursive_patchwork_lib
)

if(CUDA_FOUND)
    target_link_libraries(recursive_patchwork_node 
        ${CUDA_LIBRARIES}
        cuda_kernels_lib
    )
endif()

# Add ROS2 dependencies using ament_target_dependencies
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

# CUDA test executable
add_executable(test_cuda src/test_cuda.cpp)

# Link libraries for test executable
target_link_libraries(test_recursive_patchwork
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
    recursive_patchwork_lib
)

if(CUDA_FOUND)
    target_link_libraries(test_recursive_patchwork 
        ${CUDA_LIBRARIES}
        cuda_kernels_lib
    )
endif()

# Link libraries for CUDA test executable
target_link_libraries(test_cuda
    recursive_patchwork_lib
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
)

if(CUDA_FOUND)
    target_link_libraries(test_cuda 
        ${CUDA_LIBRARIES}
        cuda_kernels_lib
    )
endif()

# Add ROS2 libraries to test
ament_target_dependencies(test_recursive_patchwork
    rclcpp
    sensor_msgs
    rosbag2_cpp
)

# Add ROS2 libraries to CUDA test
ament_target_dependencies(test_cuda
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

# Compiler flags for CUDA test executable
target_compile_options(test_cuda PRIVATE
    -O2
    -Wall
    -Wextra
    -g
)

# Install targets
install(TARGETS 
    recursive_patchwork_lib
    recursive_patchwork 
    recursive_patchwork_node
    test_recursive_patchwork
    test_cuda
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
)

if(CUDA_FOUND)
    install(TARGETS 
        cuda_kernels_lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
    )
endif()

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
ament_export_libraries(recursive_patchwork_lib)
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