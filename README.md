# Recursive Patchwork C++ Implementation

![Recursive Patchwork Demo](result.gif)
*Patchwork, Patchwork++, & Recursive Patchwork vs. raw lidar data from KITTI-360.*

We are introducing a new set of modifications to patchwork ground segmentation algorithms. Tests on spinning lidar from a car in a domestic driving situation (shown above) showed promise to detect vehicles, and this may improve object detection, tracking, and state estimation in multiagent driving. Further testing from data coming from 3 solid state lidars on a high speed IAC-AV-24 racing car showed promising results as well. 

## Creators

- **Philip Naveen**: Created the algorithm, evaluation, & ROS2 dev.
- **Srinivasan Kannan**: Code testing, evaluation, & manuscript.

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

