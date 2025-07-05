# Recursive Patchwork C++ Implementation

A high-performance C++ implementation of the Recursive Patchwork algorithm for LiDAR point cloud processing.

## Demo

![Recursive Patchwork Demo](result.gif)

*Demo showing ground segmentation and obstacle detection using the Recursive Patchwork algorithm*

## Features

- **Recursive Patchwork Algorithm**: Advanced ground segmentation using recursive plane fitting
- **ROS2 Integration**: Native support for ROS2 bag files (DB3 and MCAP formats)
- **Standalone Mode**: Can run without ROS2 for testing and development
- **Comprehensive Testing**: Simple test suite with synthetic data generation

## Dependencies

### Required Dependencies
- **CMake** (>= 3.16)
- **C++17** compatible  compiler (g++ >= 7, Clang >= 5, MSVC >= 2017)
- **Eigen3** (>= 3.3.0)
- **OpenCV** (>= 4.0.0)

### Optional Dependencies
- **CUDA** (>= 10.0) - for GPU acceleration. We haven't made this part yet, but it's coming once I get better t CUDA.
- **ROS2** (>= Humble) - for ROS2 bag file support

