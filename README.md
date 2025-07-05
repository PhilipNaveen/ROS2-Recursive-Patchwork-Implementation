# Recursive Patchwork C++ Implementation

A high-performance C++ implementation of the Recursive Patchwork algorithm for LiDAR point cloud processing, featuring multi-LiDAR fusion, recursive ground segmentation, and Bird's Eye View (BEV) visualization.

## Features

- **Recursive Patchwork Algorithm**: Advanced ground segmentation using recursive plane fitting
- **Multi-LiDAR Fusion**: Seamless integration of multiple LiDAR sensors
- **ROS2 Integration**: Native support for ROS2 bag files (DB3 and MCAP formats)
- **Standalone Mode**: Can run without ROS2 for testing and development
- **BEV Visualization**: OpenCV-based Bird's Eye View image generation
- **CUDA Acceleration**: Optional GPU acceleration for large point clouds
- **Comprehensive Testing**: Full test suite with synthetic data generation

## Dependencies

### Required Dependencies
- **CMake** (>= 3.16)
- **C++17** compatible compiler (GCC >= 7, Clang >= 5, MSVC >= 2017)
- **Eigen3** (>= 3.3.0)
- **OpenCV** (>= 4.0.0)

### Optional Dependencies
- **CUDA** (>= 10.0) - for GPU acceleration
- **ROS2** (>= Humble) - for ROS2 bag file support

## Installation

### Ubuntu/Debian

```bash
# Install system dependencies
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libopencv-dev \
    pkg-config

# Optional: Install CUDA for GPU acceleration
# sudo apt install -y nvidia-cuda-toolkit

# Optional: Install ROS2 for bag file support
# Follow ROS2 installation guide: https://docs.ros.org/en/humble/Installation.html
```

### macOS

```bash
# Using Homebrew
brew install cmake eigen opencv pkg-config

# Optional: Install CUDA
# brew install cuda
```

### Windows

```bash
# Using vcpkg
vcpkg install eigen3 opencv4

# Or using MSYS2/MinGW
pacman -S mingw-w64-x86_64-cmake mingw-w64-x86_64-eigen3 mingw-w64-x86_64-opencv
```

## Building

### Standalone Build (Recommended for Testing)

```bash
# Clone the repository
git clone <repository-url>
cd Recursive-Patchwork

# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Build the project
make -j$(nproc)  # Use all available cores
# or
make -j4         # Use 4 cores
```

### ROS2 Build (If ROS2 is installed)

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash  # Adjust for your ROS2 version

# Build as ROS2 package
mkdir build && cd build
cmake .. -DUSE_ROS2=ON
make -j$(nproc)
```

### Build Options

```bash
# Enable CUDA acceleration
cmake .. -DUSE_CUDA=ON

# Enable ROS2 support
cmake .. -DUSE_ROS2=ON

# Set build type
cmake .. -DCMAKE_BUILD_TYPE=Release  # or Debug, RelWithDebInfo

# Install to system
make install
```

## Testing

### 1. Run the Test Suite

```bash
cd build
./test_recursive_patchwork
```

This will run comprehensive tests including:
- Basic functionality tests
- Enhanced filtering tests
- Point cloud processor tests
- LiDAR fusion tests
- Performance benchmarks

### 2. Demo Mode (Standalone)

```bash
cd build
./recursive_patchwork --demo --use-patchwork
```

This generates synthetic point cloud data and processes it with the Recursive Patchwork algorithm.

### 3. Demo with Custom Parameters

```bash
cd build
./recursive_patchwork --demo \
    --bev-width 600 \
    --bev-height 300 \
    --use-patchwork \
    --separate-display \
    --target-height 1.1 \
    --height-tolerance 0.5
```

### 4. Process ROS2 Bag Files (If ROS2 is available)

```bash
cd build
./recursive_patchwork /path/to/your/bag_file.db3 \
    --topics /lidar_front /lidar_left \
    --frame 10 \
    --use-patchwork \
    --bev-width 600 \
    --bev-height 300
```

## Usage Examples

### Command Line Options

```bash
./recursive_patchwork [options]

Options:
  --demo                         Run demo with synthetic data
  --topics <topic1> [topic2] ... Topic names (default: /lidar_points)
  --frame <number>              Frame number (default: 0)
  --bev-width <pixels>          BEV image width (default: 300)
  --bev-height <pixels>         BEV image height (default: 150)
  --x-min <meters>              Minimum X coordinate (default: -150)
  --y-min <meters>              Minimum Y coordinate (default: -75)
  --use-patchwork               Use Recursive Patchwork filtering
  --target-height <meters>      Target height for obstacles (default: 1.1)
  --height-tolerance <meters>   Height tolerance (default: 0.5)
  --separate-display            Display ground and non-ground separately
  --help                        Show this help message
```

### Example Workflows

#### 1. Quick Demo Test
```bash
./recursive_patchwork --demo --use-patchwork
# Generates: demo_frame_patchwork.png
```

#### 2. High-Resolution Visualization
```bash
./recursive_patchwork --demo \
    --bev-width 800 \
    --bev-height 400 \
    --use-patchwork \
    --separate-display
# Generates: demo_frame_patchwork.png, demo_frame_enhanced.png
```

#### 3. ROS2 Bag Processing
```bash
./recursive_patchwork data.bag \
    --topics /lidar_front /lidar_rear \
    --frame 25 \
    --use-patchwork \
    --bev-width 600 \
    --bev-height 300
# Generates: lidar_bev_frame_25_patchwork.png
```

## Output Files

The program generates PNG images showing:
- **Original**: Raw point cloud visualization
- **Patchwork**: Ground vs non-ground separation
- **Enhanced**: Height-filtered obstacle detection

## Performance

Typical performance on modern hardware:
- **10,000 points**: ~5-10ms processing time
- **100,000 points**: ~50-100ms processing time
- **1,000,000 points**: ~500ms-1s processing time

With CUDA acceleration, performance can be 2-5x faster for large point clouds.

## Configuration

### PatchworkConfig Parameters

```cpp
PatchworkConfig config;
config.sensor_height = 1.2f;        // LiDAR sensor height above ground
config.filtering_radius = 50.0f;    // Maximum processing radius
config.num_sectors = 8;             // Number of ring sectors
config.max_iter = 50;               // Maximum iterations for plane fitting
```

### Visualization Settings

```cpp
// BEV image settings
int width = 600;                    // Image width in pixels
int height = 300;                   // Image height in pixels
float x_min = -150.0f;              // Minimum X coordinate
float y_min = -75.0f;               // Minimum Y coordinate
float x_max = 150.0f;               // Maximum X coordinate
float y_max = 75.0f;                // Maximum Y coordinate
```

## Troubleshooting

### Common Build Issues

1. **Eigen3 not found**:
   ```bash
   sudo apt install libeigen3-dev
   ```

2. **OpenCV not found**:
   ```bash
   sudo apt install libopencv-dev
   ```

3. **CMake version too old**:
   ```bash
   # Install newer CMake
   sudo apt install cmake3
   # or download from cmake.org
   ```

### Runtime Issues

1. **No ROS2 support**: Use `--demo` mode for testing
2. **Memory issues**: Reduce point cloud size or enable CUDA
3. **Slow performance**: Enable CUDA acceleration or optimize parameters

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Ensure all tests pass
6. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Original Recursive Patchwork algorithm research
- ROS2 community for bag file support
- OpenCV for visualization capabilities
- Eigen3 for efficient linear algebra operations
