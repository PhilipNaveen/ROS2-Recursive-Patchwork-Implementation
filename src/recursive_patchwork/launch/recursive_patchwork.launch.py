#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        description='Path to ROS2 bag file'
    )
    
    topics_arg = DeclareLaunchArgument(
        'topics',
        default_value='/lidar_points',
        description='Comma-separated list of point cloud topics'
    )
    
    frame_arg = DeclareLaunchArgument(
        'frame',
        default_value='0',
        description='Frame number to process'
    )
    
    use_patchwork_arg = DeclareLaunchArgument(
        'use_patchwork',
        default_value='true',
        description='Use Recursive Patchwork filtering'
    )
    
    # Get launch configurations
    bag_file = LaunchConfiguration('bag_file')
    topics = LaunchConfiguration('topics')
    frame = LaunchConfiguration('frame')
    use_patchwork = LaunchConfiguration('use_patchwork')
    
    # Create the recursive patchwork node
    recursive_patchwork_node = Node(
        package='recursive_patchwork',
        executable='recursive_patchwork',
        name='recursive_patchwork_node',
        output='screen',
        arguments=[
            bag_file,
            '--topics', topics,
            '--frame', frame,
            '--use-patchwork' if use_patchwork == 'true' else '',
            '--bev-width', '600',
            '--bev-height', '300'
        ],
        parameters=[{
            'use_patchwork': use_patchwork,
            'bev_width': 600,
            'bev_height': 300
        }]
    )
    
    return LaunchDescription([
        bag_file_arg,
        topics_arg,
        frame_arg,
        use_patchwork_arg,
        recursive_patchwork_node
    ]) 