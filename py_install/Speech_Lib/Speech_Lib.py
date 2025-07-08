#!/usr/bin/env python3
# coding: utf-8
import time
import sys
import serial

class Speech(object):
    def __init__(self, com="/dev/myspeech"):
        try:
            self.ser = serial.Serial(com, 115200, timeout=0.1)
            if self.ser.isOpen():
                print(f"Speech Serial Opened! Port={com}, Baudrate=115200")
            else:
                print(f"Speech Serial Open Failed! Port={com}")
                sys.exit(1)
        except serial.SerialException as e:
            print(f"串口打开错误: {str(e)}")
            sys.exit(1)
    
    def __del__(self):
        if self.ser.isOpen():
            self.ser.close()
        print("Speech serial closed!")
    
    # 选择播报语句（如需使用）
    def send_command(self, cmd_value):
        """发送协议命令到语音模块（根据实际协议格式）"""
        # 假设协议格式为 AA 55 00 XX FB
        cmd = [0xAA, 0x55, 0x00, cmd_value, 0xFB]
        print(f"发送协议: {[hex(x) for x in cmd]}")
        self.ser.write(cmd)
        time.sleep(0.01)
        self.ser.flushInput()
    
    # 读取识别的语音命令
    def speech_read(self):
        try:
            count = self.ser.inWaiting()
            if count >= 5:  # 假设实际协议为5字节：AA 55 00 XX FB
                speech_data = self.ser.read(5)
                hex_data = speech_data.hex()
                
                # 检查协议头和尾
                if hex_data.startswith('aa55') and hex_data.endswith('fb'):
                    # 命令值在第4字节（索引6:8）
                    cmd_value = int(hex_data[6:8], 16)
                    print(f"解析协议: {hex_data}, 命令值: {cmd_value}")
                    self.ser.flushInput()
                    return cmd_value
                else:
                    print(f"无效协议格式: {hex_data}")
            
            # 处理不完整数据
            if count > 0:
                print(f"接收不完整数据: {self.ser.read(count).hex()}")
            
            self.ser.flushInput()
        except Exception as e:
            print(f"读取串口错误: {str(e)}")
        
        return 999  # 无有效数据