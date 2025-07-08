#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.voice_pub = self.create_publisher(String, '/voice_command', 10)
        
        self.obstacle_detected = False
        self.safe_distance = 0.3  # 30cm
        self.angle_range = 30  # degrees (±30°)
        
        self.get_logger().info("障碍物检测模块初始化完成")

    def scan_callback(self, msg):
        # 转换角度范围为弧度
        min_angle = -self.angle_range * 3.14159 / 180
        max_angle = self.angle_range * 3.14159 / 180
        
        # 查找对应的索引
        min_index = int((min_angle - msg.angle_min) / msg.angle_increment)
        max_index = int((max_angle - msg.angle_min) / msg.angle_increment)
        
        # 检查前方扇区范围
        obstacle_found = False
        for i in range(min_index, max_index):
            if i < len(msg.ranges) and msg.ranges[i] < self.safe_distance:
                obstacle_found = True
                break
        
        # 发布障碍物状态
        obstacle_msg = Bool()
        obstacle_msg.data = obstacle_found
        self.obstacle_pub.publish(obstacle_msg)
        
        # 处理状态变化
        if obstacle_found and not self.obstacle_detected:
            # 检测到障碍物 - 发送语音指令
            voice_msg = String()
            voice_msg.data = "obstacle_ahead"
            self.voice_pub.publish(voice_msg)
            self.get_logger().warn("检测到障碍物! 停止移动。")
        
        elif not obstacle_found and self.obstacle_detected:
            # 路径清除 - 发送清除指令
            voice_msg = String()
            voice_msg.data = "path_clear"
            self.voice_pub.publish(voice_msg)
            self.get_logger().info("路径已清除。恢复导航。")
        
        self.obstacle_detected = obstacle_found

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()