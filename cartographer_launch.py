from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'tracking_frame': 'laser',
                'published_frame': 'laser'
            }],
            arguments=[
                '-configuration_directory', '/opt/ros/humble/share/cartographer_ros/configuration_files',
                '-configuration_basename', 'revo_lds.lua'
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05, 'use_sim_time': False}]
        )
    ])