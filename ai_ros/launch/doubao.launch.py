from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_ros',
            executable='doubao.py',
            name='traffic_light_detector',
            output='screen',
            parameters=[{
                'serial_port': '/dev/myspeech',
                'max_send_times': 100
            }]
        )
    ])