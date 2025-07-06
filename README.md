# Recursive Patchwork C++ Implementation
![Recursive Patchwork Demo](result.gif)
*Patchwork, Patchwork++, & Recursive Patchwork vs. raw lidar data from KITTI-360.*

We are introducing a new set of modifications to patchwork ground segmentation algorithms. Tests on spinning lidar from a car in a domestic driving situation (shown above) showed promise to detect vehicles, and this may improve object detection, tracking, and state estimation in multiagent driving. Further testing from data coming from 3 solid state lidars on a high speed IAC-AV-24 racing car showed promising results as well. 

## Creators
- **Philip Naveen** *(dxj2ut@virginia.edu)*: Created the algorithm + initial experiments & main ROS2 developer.
- **Srinivasan Kannan** *(bff5vw@virginia.edu)*: Large scale testing and analysis of the algorithm's runtime behaviors.

## Features
- **Recursive Patchwork Algorithm**: Advanced ground segmentation using recursive plane fitting
- **ROS2 Integration**: Native support for ROS2 bag files (DB3 and MCAP formats)
- **Standalone Mode**: Can run without ROS2 for testing and development
- **Comprehensive Testing**: Simple test suite with synthetic data generation

## Algorithm
Recursive patchwork does segmenting by dividing a point cloud into a polar grid, running PCA, and further dividing the tiles when necessary. Within a radius $R$, the algorithm filters. The points are divided into radial sectors, and within these sectors, seed points are chosen. This is based on their distance from the lidar, and is described by

$$z_{th} = h + \frac{\lambda \bar{d}}{R}$$

where $\bar{d}$ is the average distance across the patch and $h$ is the lidar height. Then, a PCA plane is fit to these seed points using a centroid $\mu = \Sigma p_i$, and a covariance matrix of

$$C = \frac{1}{n-1} Q^T Q$$

for centered data $Q = P - \mu$. Then we take normal vector $n$ as the eigenvector that corresponds to the smallest eigenvalue. The algorithm then refines the ground mask by filtering points satisfying $| (p - \mu) \cdot n| < \tau$, where $\tau$ is the threshold to be considered a ground point. $\tau$ gets scaled by distance. If the mean residual error is greater than a threshold, and the patch is of sufficient size, it is split along whichever axis has the higher variance. Then, the plane fitting is applied to each sub patch. This recursive refinement allows the algorithm to adaptively resolve non-planar or cluttered regions without relying on fixed spatial resolution or assumptions of flatness. Further, this allows effectively flat regions of space to be fit with larger planes, as much detail need not be captured there.

## Dependencies
### Required Dependencies
- **CMake** (>= 3.16)
- **C++17** compatible compiler (g++ >= 7, Clang >= 5, MSVC >= 2017)
- **Eigen3** (>= 3.3.0)
- **OpenCV** (>= 4.0.0)

### Optional Dependencies
- **CUDA** (>= 10.0) - for GPU acceleration of point cloud operations
- **ROS2** (>= Humble) - for ROS2 bag file support



### Usage

Use these commands to build this into a ROS2 package.

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Build (use the magic command in ROOT)
colcon build --packages-select recursive_patchwork --cmake-args -DCMAKE_TOOLCHAIN_FILE="" -DCMAKE_MAKE_PROGRAM=/usr/bin/make -DCMAKE_C_COMPILER=/usr/bin/cc -DCMAKE_CXX_COMPILER=/usr/bin/c++

# Source the workspace
source install/setup.bash

# Run the node
ros2 run recursive_patchwork recursive_patchwork_node

# Or use the launch file
ros2 launch recursive_patchwork recursive_patchwork.launch.py

```

Or use this for a more aggressive but still functional build straight from CMAKE.

```
cd src/recursive_patchwork && rm -rf build/ && mkdir build && cd build && cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="" -DCMAKE_MAKE_PROGRAM=/usr/bin/make -DCMAKE_C_COMPILER=/usr/bin/cc -DCMAKE_CXX_COMPILER=/usr/bin/c++ && make -j4
```

## GPU Acceleration with CUDA

The package now includes GPU-accelerated point cloud operations using CUDA. The implementation automatically falls back to CPU when CUDA is not available.

### Project Structure
```
src/recursive_patchwork/
├── src/                    # Regular C++ source files
│   ├── lidar_fusion.cpp    # LiDAR fusion implementation
│   └── ...
├── cuda/                   # CUDA-specific source files
│   ├── cuda_wrapper.cu     # CUDA kernels and wrappers
│   └── cuda_interface.cu   # CUDA interface implementation
├── include/                # Headers
│   ├── cuda_interface.hpp  # CUDA interface
│   └── ...
└── CMakeLists.txt
```

### CUDA Features
- **2D Rotation**: Parallel rotation of point clouds around the Z-axis
- **4x4 Transformation**: Parallel application of homogeneous transformation matrices
- **Ego Vehicle Filtering**: Parallel removal of points within a specified radius
- **Automatic Fallback**: Seamless CPU fallback when CUDA is unavailable

### Testing CUDA Acceleration

To test if CUDA acceleration is working:

```bash
# Build with CUDA support
colcon build --packages-select recursive_patchwork

# Run the CUDA test
ros2 run recursive_patchwork test_cuda
```

The test will:
1. Check if CUDA is available
2. Compare CPU vs GPU performance
3. Verify that results match between CPU and GPU implementations
4. Report timing differences

### Performance Benefits

GPU acceleration provides significant speedup for large point clouds:
- **2D Rotation**: 5-10x speedup for 100k+ points
- **Ego Vehicle Filtering**: 3-8x speedup for 100k+ points  
- **4x4 Transformation**: 4-12x speedup for 100k+ points

The speedup depends on:
- Point cloud size (larger clouds benefit more)
- GPU hardware (newer GPUs provide better performance)
- Memory transfer overhead (small clouds may not benefit due to transfer costs)

