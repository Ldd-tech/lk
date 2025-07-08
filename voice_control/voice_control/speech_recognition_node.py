#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time

# 添加Speech_Lib库的路径
sys.path.append('/home/sunrise/robot_ws/src/py_install/Speech_Lib')
from Speech_Lib import Speech

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        self.get_logger().info("语音识别节点已启动")
        
        # 初始化语音识别模块
        try:
            self.speech = Speech("/dev/myspeech")
            self.get_logger().info("语音识别模块初始化成功")
        except Exception as e:
            self.get_logger().error(f"初始化失败: {str(e)}")
            raise
        
        # 创建语音命令发布者
        self.voice_publisher = self.create_publisher(String, '/voice_words', 10)
        
        # 创建定时器，定期检查语音输入（10Hz）
        self.timer = self.create_timer(0.1, self.check_speech)
        
        # 语音命令映射表（根据实际协议命令值配置）
        self.command_mapping = {
            0x01: "小车停止",    # AA 55 00 01 FB
            0x02: "停车",        # AA 55 00 02 FB
            0x03: "小车休眠",    # AA 55 00 03 FB
            0x04: "小车前进",    # AA 55 00 04 FB
            0x05: "小车后退",    # AA 55 00 05 FB
            0x06: "小车左转",    # AA 55 00 06 FB
            0x07: "小车右转",    # AA 55 00 07 FB
            0x08: "小车左旋",    # AA 55 00 08 FB
            0x09: "小车右旋",    # AA 55 00 09 FB
            0x0A: "关灯",        # AA 55 00 0A FB
            # 以下为示例，需根据完整协议表补充
            0x10: "最大音量",     # AA 55 06 00 FB（假设命令值为0x10，需根据实际协议调整）
            0x11: "中等音量",     # AA 55 07 00 FB
            # ... 补充完整所有命令 ...
        }
        
        self.get_logger().info(f"语音命令映射表加载完成，共{len(self.command_mapping)}个命令")
    
    def check_speech(self):
        """检查语音输入并发布相应命令"""
        try:
            voice_data = self.speech.speech_read()
            
            if voice_data == 999:
                return
                
            self.get_logger().info(f"检测到语音数据: 0x{voice_data:02X}")
            
            # 检查命令映射
            if voice_data in self.command_mapping:
                command = self.command_mapping[voice_data]
                msg = String()
                msg.data = command
                self.voice_publisher.publish(msg)
                self.get_logger().info(f"发布命令: {command}")
            else:
                self.get_logger().warning(f"未知命令值: 0x{voice_data:02X}")
        except Exception as e:
            self.get_logger().error(f"语音识别错误: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SpeechRecognitionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点异常: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()