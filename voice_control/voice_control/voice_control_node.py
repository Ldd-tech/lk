#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.get_logger().info("语音控制节点已启动")
        
        # 创建速度命令发布者
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 创建语音命令订阅者
        self.create_subscription(
            String,
            '/voice_words',
            self.voice_command_callback,
            10
        )
        
        # 命令映射字典
        self.command_mapping = {
            "小车前进": self.move_forward,
            "小车后退": self.move_backward,
            "小车停止": self.stop,
            "停车": self.stop,
            "增大音量": self.increase_volume,
            "减小音量": self.decrease_volume,
            "最大音量": self.max_volume,
            "中等音量": self.medium_volume,
            "最小音量": self.min_volume,
            "开启播报": self.enable_announcement,
            "关闭播报": self.disable_announcement
        }
        
        # 当前运动状态
        self.current_motion = "stop"
        
        # 创建定时器，持续发布速度命令
        self.timer = self.create_timer(0.1, self.publish_velocity)  # 10Hz
    
    def publish_velocity(self):
        """根据当前状态持续发布速度命令"""
        twist = Twist()
        
        if self.current_motion == "forward":
            twist.linear.x = 0.2
        elif self.current_motion == "backward":
            twist.linear.x = -0.2
        else:  # stop
            twist.linear.x = 0.0
        
        self.cmd_vel_publisher.publish(twist)
    
    def voice_command_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f"收到语音命令: {command}")
        
        # 检查命令是否在映射中
        if command in self.command_mapping:
            self.command_mapping[command]()
            self.get_logger().info(f"执行命令: {command}")
        else:
            self.get_logger().warning(f"未知命令: {command}")
    
    def move_forward(self):
        self.current_motion = "forward"
    
    def move_backward(self):
        self.current_motion = "backward"
    
    def stop(self):
        self.current_motion = "stop"
    
    def increase_volume(self):
        self.get_logger().info("执行: 增大音量")
    
    def decrease_volume(self):
        self.get_logger().info("执行: 减小音量")
    
    def max_volume(self):
        self.get_logger().info("执行: 最大音量")
    
    def medium_volume(self):
        self.get_logger().info("执行: 中等音量")
    
    def min_volume(self):
        self.get_logger().info("执行: 最小音量")
    
    def enable_announcement(self):
        self.get_logger().info("执行: 开启播报")
    
    def disable_announcement(self):
        self.get_logger().info("执行: 关闭播报")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()