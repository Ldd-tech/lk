#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import subprocess
import os
import time
import cv2
from pyzbar import pyzbar
import numpy as np
import threading
import logging

class VoicePaymentNode(Node):
    def __init__(self):
        super().__init__('voice_payment_node')
        self.get_logger().info("启动语音支付节点...")
        
        # 配置参数
        self.declare_parameter('serial_port', '/dev/myspeech')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('qr_img_path', '/home/sunrise/payment_qr.png')
        self.declare_parameter('display_timeout', 30)  # 二维码显示超时时间（秒）
        self.declare_parameter('camera_index', 1)      # 摄像头索引
        self.declare_parameter('scan_timeout', 10)     # 扫描超时时间（秒）
        self.declare_parameter('preview_size', (800, 600))  # 预览窗口大小
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.qr_img_path = os.path.expanduser(self.get_parameter('qr_img_path').value)
        self.display_timeout = self.get_parameter('display_timeout').value
        self.camera_index = self.get_parameter('camera_index').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.preview_size = self.get_parameter('preview_size').value
        
        # 初始化串口
        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.1
            )
            self.get_logger().info(f"已连接语音模块: {serial_port} @ {baud_rate}bps")
        except Exception as e:
            self.get_logger().error(f"串口连接失败: {str(e)}")
            raise
        
        # 创建定时器检查串口数据
        self.timer = self.create_timer(0.05, self.check_serial)
        
        # 当前显示进程
        self.display_process = None
        self.last_display_time = 0
        self.get_logger().info(f"等待付款码指令... (二维码路径: {self.qr_img_path})")
        
        # 初始化摄像头 (RGB摄像头)
        self.camera = cv2.VideoCapture(self.camera_index)
        if not self.camera.isOpened():
            self.get_logger().error("无法打开摄像头，请检查连接")
        else:
            # 设置摄像头分辨率 (RGB摄像头通常支持更高分辨率)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            self.get_logger().info(f"RGB摄像头已初始化 (索引: {self.camera_index}, 分辨率: 1920x1080)")
            
            # 启动摄像头预览
            self.camera_preview_thread = threading.Thread(target=self.preview_camera)
            self.camera_preview_thread.daemon = True
            self.camera_preview_thread.start()
            self.get_logger().info("摄像头预览已启动")
            
        # 创建扫描状态变量
        self.scanning = False
        self.scan_thread = None

        # 协议映射表 (根据最新协议表更新)
        self.protocol_actions = {
            # 付款码相关协议
            b'\xAA\x55\x00\x2F\xFB': ("请出示付款码", self.display_qr_code),        # 显示付款码
            b'\xAA\x55\x00\x2E\xFB': ("关闭付款码", self.close_display),           # 关闭付款码
            b'\xAA\x55\x00\x30\xFB': ("扫描付款码", self.scan_payment_code),       # 扫描付款码
            
            # 其他常用协议
            b'\xAA\x55\x00\x0A\xFB': ("关闭播报", self.close_display),             # 关闭播报
            b'\xAA\x55\x03\x00\xFB': ("唤醒响应", None)                            # 唤醒响应
        }
        
        # 摄像头预览控制
        self.preview_running = True

    def preview_camera(self):
        """实时显示摄像头预览画面"""
        window_name = "摄像头预览 (按ESC退出)"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, self.preview_size[0], self.preview_size[1])
        
        try:
            while self.preview_running:
                # 读取摄像头帧
                ret, frame = self.camera.read()
                if not ret:
                    self.get_logger().warning("预览时无法从摄像头获取帧")
                    time.sleep(0.1)
                    continue
                
                # 缩小图像以降低处理负担
                preview_frame = cv2.resize(frame, self.preview_size)
                
                # 显示帧
                cv2.imshow(window_name, preview_frame)
                
                # 检查按键 (ESC键退出)
                key = cv2.waitKey(1)
                if key == 27:  # ESC键
                    self.get_logger().info("用户按ESC键关闭预览")
                    break
                    
        except Exception as e:
            self.get_logger().error(f"摄像头预览错误: {str(e)}")
        finally:
            cv2.destroyWindow(window_name)
            self.get_logger().info("摄像头预览已关闭")

    def check_serial(self):
        """检查串口数据并处理协议"""
        try:
            # 读取完整协议帧 (5字节)
            if self.ser.in_waiting >= 5:
                data = self.ser.read(5)
                
                # 调试输出
                hex_data = ' '.join(f'{b:02X}' for b in data)
                self.get_logger().debug(f"收到协议: {hex_data}")
                
                # 检查支持的协议
                if data in self.protocol_actions:
                    action_name, action_func = self.protocol_actions[data]
                    self.get_logger().info(f"执行指令: {action_name}")
                    
                    if action_func:
                        action_func()
                
                # 检查超时自动关闭
                self.check_display_timeout()
        except Exception as e:
            self.get_logger().error(f"串口处理错误: {str(e)}")

    def display_qr_code(self):
        """显示付款码"""
        try:
            # 关闭可能已存在的显示
            self.close_display()
            
            # 使用feh全屏显示二维码
            self.display_process = subprocess.Popen(
                ['feh', '--fullscreen', self.qr_img_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.last_display_time = time.time()
            self.get_logger().info(f"付款码已显示: {self.qr_img_path}")
            
            # 发送语音反馈 (主动播报协议)
            self.send_voice_feedback(b'\xAA\x55\x00\x2F\xFB')  # 发送"请出示付款码"协议
        except Exception as e:
            self.get_logger().error(f"显示失败: {str(e)}")

    def close_display(self):
        """关闭当前显示"""
        if self.display_process and self.display_process.poll() is None:
            self.display_process.terminate()
            self.display_process.wait()
            self.get_logger().info("已关闭付款码显示")
            
            # 发送语音反馈 (主动播报协议)
            self.send_voice_feedback(b'\xAA\x55\x00\x2E\xFB')  # 发送"关闭付款码"协议
        self.display_process = None

    def scan_payment_code(self):
        """扫描付款码"""
        if self.scanning:
            self.get_logger().warning("扫描操作正在进行中，忽略新请求")
            return
            
        # 启动扫描线程
        self.scanning = True
        self.scan_thread = threading.Thread(target=self.scan_qr_code)
        self.scan_thread.daemon = True
        self.scan_thread.start()
        self.get_logger().info("启动付款码扫描线程")
        
        # 发送开始扫描提示 (主动播报协议)
        self.send_voice_feedback(b'\xAA\x55\x00\x30\xFB')  # 发送"扫描付款码"协议

    def scan_qr_code(self):
        """扫描二维码并处理结果"""
        self.get_logger().info("开始扫描付款码...")
        
        try:
            # 设置扫描超时
            start_time = time.time()
            timeout = self.scan_timeout
            scan_success = False
            
            while self.scanning and (time.time() - start_time < timeout):
                # 读取摄像头帧 (RGB格式)
                ret, frame = self.camera.read()
                if not ret:
                    self.get_logger().warning("无法从摄像头获取帧")
                    time.sleep(0.1)
                    continue
                
                # 转换为灰度图像并检测二维码
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                barcodes = pyzbar.decode(gray)
                
                # 处理检测到的二维码
                if barcodes:
                    for barcode in barcodes:
                        # 提取二维码数据
                        barcode_data = barcode.data.decode("utf-8")
                        self.get_logger().info(f"扫描到付款码: {barcode_data}")
                        
                        # 发送识别成功反馈 (被动播报协议)
                        self.send_voice_feedback(b'\xAA\x55\xFF\x5F\xFB')  # 发送"检测到收款码"协议
                        
                        # 显示扫描结果
                        self.display_scan_result(frame, barcode)
                        
                        scan_success = True
                        break
            
            # 扫描结束处理
            if not scan_success:
                self.get_logger().warning("未检测到付款码")
                # 发送识别失败反馈 (被动播报协议)
                self.send_voice_feedback(b'\xAA\x55\xFF\x62\xFB')  # 发送"未检测到收款码"协议

        except Exception as e:
            self.get_logger().error(f"扫描过程中出错: {str(e)}")
        finally:
            self.scanning = False
            self.get_logger().info("付款码扫描结束")

    def display_scan_result(self, frame, barcode):
        """在图像上显示扫描结果并保存"""
        try:
            # 提取二维码位置
            (x, y, w, h) = barcode.rect
            
            # 在RGB图像上绘制矩形
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            
            # 在图像上显示二维码数据
            text = f"Payment Code: {barcode.data.decode('utf-8')[:20]}..."
            cv2.putText(frame, text, (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            
            # 保存扫描结果图像
            result_dir = "/home/sunrise/scan_results"
            os.makedirs(result_dir, exist_ok=True)
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            result_path = f"{result_dir}/scan_result_{timestamp}.jpg"
            cv2.imwrite(result_path, frame)
            self.get_logger().info(f"扫描结果已保存至: {result_path}")
            
            # 显示扫描结果 (可选)
            subprocess.Popen(['feh', result_path])
            
        except Exception as e:
            self.get_logger().error(f"处理扫描结果出错: {str(e)}")

    def send_voice_feedback(self, protocol):
        """通过语音模块发送反馈协议"""
        try:
            self.ser.write(protocol)
            hex_protocol = ' '.join(f'{b:02X}' for b in protocol)
            self.get_logger().info(f"发送语音反馈: {hex_protocol}")
        except Exception as e:
            self.get_logger().error(f"发送语音反馈失败: {str(e)}")

    def check_display_timeout(self):
        """检查并关闭超时显示的二维码"""
        if self.display_process and self.display_process.poll() is None:
            elapsed = time.time() - self.last_display_time
            if elapsed > self.display_timeout:
                self.get_logger().info(f"二维码已显示超过 {self.display_timeout} 秒，自动关闭")
                self.close_display()

    def __del__(self):
        """清理资源"""
        # 关闭串口
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        
        # 关闭显示
        self.close_display()
        
        # 停止扫描线程
        self.scanning = False
        if self.scan_thread and self.scan_thread.is_alive():
            self.scan_thread.join(timeout=1.0)
        
        # 释放摄像头
        if hasattr(self, 'camera') and self.camera.isOpened():
            self.camera.release()
        
        # 停止预览
        self.preview_running = False
        if self.camera_preview_thread and self.camera_preview_thread.is_alive():
            self.camera_preview_thread.join(timeout=1.0)
        
        # 关闭所有OpenCV窗口
        cv2.destroyAllWindows()
        
        self.get_logger().info("资源已释放")

def main(args=None):
    # 配置日志格式
    logging.basicConfig(
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        level=logging.INFO
    )
    
    rclpy.init(args=args)
    node = VoicePaymentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被手动终止")
    except Exception as e:
        node.get_logger().error(f"节点异常: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("节点已关闭")

if __name__ == '__main__':
    main()