#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'input_topic',
            default_value='/lidar/points',
            description='Input point cloud topic'
        ),
        DeclareLaunchArgument(
            'ground_topic',
            default_value='/patchwork/ground',
            description='Output ground points topic'
        ),
        DeclareLaunchArgument(
            'obstacles_topic',
            default_value='/patchwork/obstacles',
            description='Output obstacle points topic'
        ),
        DeclareLaunchArgument(
            'visualization_topic',
            default_value='/patchwork/visualization',
            description='Output visualization markers topic'
        ),
        DeclareLaunchArgument(
            'min_points',
            default_value='100',
            description='Minimum points for plane fitting'
        ),
        DeclareLaunchArgument(
            'max_iterations',
            default_value='50',
            description='Maximum iterations for plane fitting'
        ),
        DeclareLaunchArgument(
            'distance_threshold',
            default_value='0.1',
            description='Distance threshold for plane fitting'
        ),
        DeclareLaunchArgument(
            'angle_threshold',
            default_value='0.1',
            description='Angle threshold for plane fitting'
        ),
        
        # Recursive Patchwork Node
        Node(
            package='recursive_patchwork',
            executable='recursive_patchwork_node',
            name='recursive_patchwork_node',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'ground_topic': LaunchConfiguration('ground_topic'),
                'obstacles_topic': LaunchConfiguration('obstacles_topic'),
                'visualization_topic': LaunchConfiguration('visualization_topic'),
                'min_points': LaunchConfiguration('min_points'),
                'max_iterations': LaunchConfiguration('max_iterations'),
                'distance_threshold': LaunchConfiguration('distance_threshold'),
                'angle_threshold': LaunchConfiguration('angle_threshold'),
            }],
            remappings=[
                ('/lidar/points', LaunchConfiguration('input_topic')),
                ('/patchwork/ground', LaunchConfiguration('ground_topic')),
                ('/patchwork/obstacles', LaunchConfiguration('obstacles_topic')),
                ('/patchwork/visualization', LaunchConfiguration('visualization_topic')),
            ]
        )
    ]) 