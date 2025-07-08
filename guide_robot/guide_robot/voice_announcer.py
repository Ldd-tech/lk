#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class VoiceAnnouncer(Node):
    def __init__(self):
        super().__init__('voice_announcer')
        self.subscription = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10)
        
        # 串口配置
        self.serial_port = "/dev/myspeech"
        self.baudrate = 115200
        self.ser = None
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate)
            if self.ser.isOpen():
                self.get_logger().info(f"语音串口已打开: {self.serial_port}, 波特率: {self.baudrate}")
            else:
                self.get_logger().error("无法打开语音串口")
        except Exception as e:
            self.get_logger().error(f"语音串口初始化失败: {str(e)}")
        
        # 完整的语音指令映射
        self.command_map = {
            # 交通灯指令 - 直接中文指令
            "红灯": [0xAA, 0x55, 0xFF, 0x66, 0xFB],    # 红灯
            "绿灯": [0xAA, 0x55, 0xFF, 0x64, 0xFB],   # 绿灯
            "黄灯": [0xAA, 0x55, 0xFF, 0x63, 0xFB],  # 黄灯
            "蓝灯": [0xAA, 0x55, 0xFF, 0x65, 0xFB],  # 蓝灯
            
            # 物体检测指令 - 英文指令
            "red_light": [0xAA, 0x55, 0xFF, 0x66, 0xFB],    # 红灯
            "green_light": [0xAA, 0x55, 0xFF, 0x64, 0xFB],   # 绿灯
            "yellow_light": [0xAA, 0x55, 0xFF, 0x63, 0xFB],  # 黄灯
            "blue_light": [0xAA, 0x55, 0xFF, 0x65, 0xFB],    # 蓝灯
            "vehicle": [0xAA, 0x55, 0xFF, 0x74, 0xFB],       # 注意车辆
            "pedestrian": [0xAA, 0x55, 0xFF, 0x8D, 0xFB],    # 注意行人
            "crosswalk": [0xAA, 0x55, 0xFF, 0x75, 0xFB],     # 识别到斑马线
            
            # 障碍物指令
            "obstacle_ahead": [0xAA, 0x55, 0xFF, 0x73, 0xFB] # 前方有障碍物
        }
        
        self.get_logger().info("语音播报模块初始化完成")

    def send_voice_command(self, command):
        if command in self.command_map and self.ser and self.ser.isOpen():
            try:
                cmd = self.command_map[command]
                self.ser.write(bytearray(cmd))
                self.get_logger().info(f"✓ 发送语音指令: {command}")
                time.sleep(0.01)
                self.ser.flushInput()
                return True
            except Exception as e:
                self.get_logger().error(f"发送指令失败: {str(e)}")
        else:
            self.get_logger().warn(f"未知语音指令: {command}")
        return False

    def command_callback(self, msg):
        self.send_voice_command(msg.data)

    def __del__(self):
        if self.ser and self.ser.isOpen():
            self.ser.close()
            self.get_logger().info("语音串口已关闭")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceAnnouncer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()