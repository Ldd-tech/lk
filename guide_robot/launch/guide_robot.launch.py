import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    guide_robot_dir = get_package_share_directory('guide_robot')
    
    # 使用正确的导航包名
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    
    # 获取导航启动文件路径
    nav_launch_path = os.path.join(wheeltec_nav_dir, 'launch', 'wheeltec_nav2.launch.py')
    
    return LaunchDescription([
        # 包含导航系统
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_path)
        ),
        
        # 障碍物检测节点
        Node(
            package='guide_robot',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen'
        ),
        
        # 语音播报节点
        Node(
            package='guide_robot',
            executable='voice_announcer',
            name='voice_announcer',
            output='screen'
        ),
        
        # 视觉检测节点
        Node(
            package='guide_robot',
            executable='object_detector',
            name='object_detector',
            output='screen'
        )
    ])