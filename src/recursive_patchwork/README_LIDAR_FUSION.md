# LiDAR Fusion Node

A ROS2 node that fuses multiple LiDAR point clouds into a single unified point cloud using TF transforms.

## Features

- **Multi-LiDAR Support**: Subscribe to N LiDAR point cloud topics
- **TF Transform Integration**: Apply transforms from `/tf_static` or dynamic TF
- **Ego Vehicle Removal**: Remove ego vehicle points from each LiDAR
- **Configurable Parameters**: Easy configuration via ROS2 parameters
- **CUDA Acceleration**: Optional GPU acceleration for point cloud processing

## Usage

### Basic Usage

```bash
# Run with default 3-LiDAR configuration
ros2 run recursive_patchwork lidar_fusion_node

# Run with custom configuration
ros2 run recursive_patchwork lidar_fusion_node --ros-args \
  -p output_frame_id:=base_link \
  -p fused_topic_name:=/fused_pointcloud \
  -p lidar_topics:='["/lidar_front", "/lidar_left", "/lidar_right"]'
```

### Using Launch File

```bash
# Run with launch file
ros2 launch recursive_patchwork lidar_fusion.launch.py

# Run with custom parameters
ros2 launch recursive_patchwork lidar_fusion.launch.py \
  output_frame_id:=base_link \
  lidar_topics:='["/lidar_front", "/lidar_left", "/lidar_right"]'
```

### Using Configuration File

```bash
# Run with YAML configuration
ros2 run recursive_patchwork lidar_fusion_node \
  --ros-args --params-file config/lidar_fusion_example.yaml
```

## Parameters

### Core Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `output_frame_id` | string | `base_link` | Output frame for fused point cloud |
| `fused_topic_name` | string | `/fused_pointcloud` | Topic name for fused point cloud |
| `use_tf_static` | bool | `true` | Use `/tf_static` for transforms |
| `transform_timeout` | double | `0.1` | Timeout for dynamic transforms (seconds) |
| `min_clouds_to_fuse` | int | `1` | Minimum clouds required for fusion |
| `remove_ego_vehicle` | bool | `true` | Remove ego vehicle from point clouds |

### LiDAR Configuration

For each LiDAR (indexed from 0):

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lidar_topics` | string[] | `["/lidar_front", "/lidar_left", "/lidar_right"]` | LiDAR topic names |
| `lidar_X_frame_id` | string | `lidar_X_link` | Frame ID for LiDAR X |
| `lidar_X_target_frame` | string | `output_frame_id` | Target frame for LiDAR X |
| `lidar_X_ego_radius` | double | `2.5` | Ego vehicle radius for LiDAR X |

## Topics

### Subscribed Topics

- `lidar_X` (sensor_msgs/PointCloud2): Individual LiDAR point clouds
- `/tf_static` (tf2_msgs/TFMessage): Static transforms (if `use_tf_static=true`)
- `/tf` (tf2_msgs/TFMessage): Dynamic transforms (if `use_tf_static=false`)

### Published Topics

- `fused_topic_name` (sensor_msgs/PointCloud2): Fused point cloud

## TF Requirements

The node requires TF transforms from each LiDAR frame to the output frame:

```
base_link
├── lidar_front_link
├── lidar_left_link
└── lidar_right_link
```

### Publishing TF Transforms

```bash
# Example: Publish static transforms
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.0 \
  --yaw 0.0 --pitch 0.0 --roll 0.0 \
  --frame-id base_link --child-frame-id lidar_front_link

ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.5 --z 0.0 \
  --yaw 1.57 --pitch 0.0 --roll 0.0 \
  --frame-id base_link --child-frame-id lidar_left_link

ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y -0.5 --z 0.0 \
  --yaw -1.57 --pitch 0.0 --roll 0.0 \
  --frame-id base_link --child-frame-id lidar_right_link
```

## Example Configuration

```yaml
lidar_fusion_node:
  ros__parameters:
    output_frame_id: "base_link"
    fused_topic_name: "/fused_pointcloud"
    use_tf_static: true
    min_clouds_to_fuse: 1
    remove_ego_vehicle: true
    
    lidar_topics: ["/lidar_front", "/lidar_left", "/lidar_right"]
    
    lidar_0_frame_id: "lidar_front_link"
    lidar_1_frame_id: "lidar_left_link"
    lidar_2_frame_id: "lidar_right_link"
    
    lidar_0_ego_radius: 2.5
    lidar_1_ego_radius: 2.5
    lidar_2_ego_radius: 2.5
```

## Building

```bash
# Build the package
colcon build --packages-select recursive_patchwork

# Source the workspace
source install/setup.bash
```

## Dependencies

- ROS2 Humble or later
- sensor_msgs
- tf2_ros
- tf2_eigen
- pcl_conversions
- PCL
- Eigen3
- OpenCV
- CUDA (optional, for GPU acceleration) 