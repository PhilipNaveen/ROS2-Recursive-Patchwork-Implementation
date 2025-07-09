#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('recursive_patchwork')
    
    # Launch arguments
    output_frame_arg = DeclareLaunchArgument(
        'output_frame_id',
        default_value='base_link',
        description='Output frame ID for fused point cloud'
    )
    
    fused_topic_arg = DeclareLaunchArgument(
        'fused_topic_name',
        default_value='/fused_pointcloud',
        description='Topic name for fused point cloud'
    )
    
    use_tf_static_arg = DeclareLaunchArgument(
        'use_tf_static',
        default_value='true',
        description='Use tf_static for transforms'
    )
    
    min_clouds_arg = DeclareLaunchArgument(
        'min_clouds_to_fuse',
        default_value='1',
        description='Minimum number of clouds required for fusion'
    )
    
    remove_ego_arg = DeclareLaunchArgument(
        'remove_ego_vehicle',
        default_value='true',
        description='Remove ego vehicle from point clouds'
    )
    
    # LiDAR topic arguments
    lidar_topics_arg = DeclareLaunchArgument(
        'lidar_topics',
        default_value='["/lidar_front", "/lidar_left", "/lidar_right"]',
        description='List of LiDAR topic names'
    )
    
    # LiDAR frame arguments
    lidar_0_frame_arg = DeclareLaunchArgument(
        'lidar_0_frame_id',
        default_value='lidar_front_link',
        description='Frame ID for first LiDAR'
    )
    
    lidar_1_frame_arg = DeclareLaunchArgument(
        'lidar_1_frame_id',
        default_value='lidar_left_link',
        description='Frame ID for second LiDAR'
    )
    
    lidar_2_frame_arg = DeclareLaunchArgument(
        'lidar_2_frame_id',
        default_value='lidar_right_link',
        description='Frame ID for third LiDAR'
    )
    
    # Ego radius arguments
    lidar_0_radius_arg = DeclareLaunchArgument(
        'lidar_0_ego_radius',
        default_value='2.5',
        description='Ego vehicle radius for first LiDAR'
    )
    
    lidar_1_radius_arg = DeclareLaunchArgument(
        'lidar_1_ego_radius',
        default_value='2.5',
        description='Ego vehicle radius for second LiDAR'
    )
    
    lidar_2_radius_arg = DeclareLaunchArgument(
        'lidar_2_ego_radius',
        default_value='2.5',
        description='Ego vehicle radius for third LiDAR'
    )
    
    # LiDAR Fusion Node
    lidar_fusion_node = Node(
        package='recursive_patchwork',
        executable='lidar_fusion_node',
        name='lidar_fusion_node',
        output='screen',
        parameters=[{
            'output_frame_id': LaunchConfiguration('output_frame_id'),
            'fused_topic_name': LaunchConfiguration('fused_topic_name'),
            'use_tf_static': LaunchConfiguration('use_tf_static'),
            'min_clouds_to_fuse': LaunchConfiguration('min_clouds_to_fuse'),
            'remove_ego_vehicle': LaunchConfiguration('remove_ego_vehicle'),
            'lidar_topics': LaunchConfiguration('lidar_topics'),
            'lidar_0_frame_id': LaunchConfiguration('lidar_0_frame_id'),
            'lidar_1_frame_id': LaunchConfiguration('lidar_1_frame_id'),
            'lidar_2_frame_id': LaunchConfiguration('lidar_2_frame_id'),
            'lidar_0_ego_radius': LaunchConfiguration('lidar_0_ego_radius'),
            'lidar_1_ego_radius': LaunchConfiguration('lidar_1_ego_radius'),
            'lidar_2_ego_radius': LaunchConfiguration('lidar_2_ego_radius'),
        }]
    )
    
    return LaunchDescription([
        output_frame_arg,
        fused_topic_arg,
        use_tf_static_arg,
        min_clouds_arg,
        remove_ego_arg,
        lidar_topics_arg,
        lidar_0_frame_arg,
        lidar_1_frame_arg,
        lidar_2_frame_arg,
        lidar_0_radius_arg,
        lidar_1_radius_arg,
        lidar_2_radius_arg,
        lidar_fusion_node
    ]) 