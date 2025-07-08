from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='voice_payment',
            executable='voice_payment_node',
            name='voice_payment',
            output='screen',
            parameters=[{
                'serial_port': '/dev/myspeech',
                'baud_rate': 115200,
                'qr_img_path': '/home/sunrise/payment_qr.png',
                'display_timeout': 30,    # 30秒后自动关闭
                'camera_index': 1,        # 默认摄像头索引
                'scan_timeout': 10        # 扫描超时10秒
            }]
        )
    ])