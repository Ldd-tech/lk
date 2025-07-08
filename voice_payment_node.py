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
import glob

class VoicePaymentNode(Node):
    def __init__(self):
        super().__init__('voice_payment_node')
        self.get_logger().info("启动语音支付节点...")
        
        # 配置参数
        self.declare_parameter('serial_port', '/dev/myspeech')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('qr_img_path', '/home/sunrise/payment_qr.png')
        self.declare_parameter('display_timeout', 30)  # 二维码显示超时时间（秒）
        self.declare_parameter('camera_device', '/dev/video0')  # 摄像头设备路径
        self.declare_parameter('scan_timeout', 10)     # 扫描超时时间（秒）
        self.declare_parameter('preview_scale', 0.5)   # 预览画面缩放比例
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.qr_img_path = os.path.expanduser(self.get_parameter('qr_img_path').value)
        self.display_timeout = self.get_parameter('display_timeout').value
        self.camera_device = self.get_parameter('camera_device').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.preview_scale = self.get_parameter('preview_scale').value
        
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
        self.get_logger().info(f"等待二维码指令... (二维码路径: {self.qr_img_path})")
        
        # 初始化摄像头
        self.camera = None
        self.initialize_camera()
        
        # 创建预览窗口
        self.preview_window_name = "Camera Preview"
        cv2.namedWindow(self.preview_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.preview_window_name, 
                         int(640 * self.preview_scale), 
                         int(480 * self.preview_scale))
        
        # 启动摄像头预览线程
        self.preview_active = True
        self.preview_thread = threading.Thread(target=self.run_camera_preview)
        self.preview_thread.daemon = True
        self.preview_thread.start()
        self.get_logger().info("摄像头预览已启动")
        
        # 创建扫描状态变量
        self.scanning = False
        self.scan_start_time = 0

        # 协议映射表 (根据最新协议表更新)
        self.protocol_actions = {
            # 二维码相关协议
            b'\xAA\x55\x00\x2F\xFB': ("请出示二维码", self.display_qr_code),        # 显示二维码
            b'\xAA\x55\x00\x2E\xFB': ("关闭二维码", self.close_display),           # 关闭二维码
            b'\xAA\x55\x00\x30\xFB': ("扫描二维码", self.scan_qr_code),            # 扫描二维码
            
            # 其他常用协议
            b'\xAA\x55\x00\x0A\xFB': ("关闭播报", self.close_display),             # 关闭播报
            b'\xAA\x55\x03\x00\xFB': ("唤醒响应", None)                            # 唤醒响应
        }

    def initialize_camera(self):
        """初始化摄像头设备"""
        # 检查设备是否存在
        if not os.path.exists(self.camera_device):
            # 尝试自动检测可用摄像头
            video_devices = glob.glob('/dev/video*')
            if video_devices:
                self.camera_device = sorted(video_devices)[0]  # 使用第一个检测到的摄像头
                self.get_logger().warning(f"配置的设备不存在，使用检测到的设备: {self.camera_device}")
            else:
                self.get_logger().error("未检测到摄像头设备")
                return
        
        # 尝试打开摄像头
        try:
            self.camera = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
            if not self.camera.isOpened():
                self.get_logger().error(f"无法打开摄像头设备: {self.camera_device}")
                return
            
            # 设置较低的分辨率（640x480 通常更兼容）
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            # 检查分辨率设置是否成功
            width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(f"摄像头已初始化 (设备: {self.camera_device}, 分辨率: {width}x{height})")
            
            # 给摄像头初始化时间
            time.sleep(2.0)
            
        except Exception as e:
            self.get_logger().error(f"摄像头初始化失败: {str(e)}")

    def run_camera_preview(self):
        """持续运行摄像头预览"""
        self.get_logger().info("摄像头预览线程启动")
        
        # 添加重试计数器
        retry_count = 0
        max_retries = 5
        
        while self.preview_active:
            try:
                # 检查摄像头是否可用
                if self.camera is None or not self.camera.isOpened():
                    self.get_logger().warning("摄像头未初始化，尝试重新初始化...")
                    self.initialize_camera()
                    if self.camera is None or not self.camera.isOpened():
                        self.get_logger().error("摄像头初始化失败，等待后重试...")
                        time.sleep(1.0)
                        continue
                
                # 读取摄像头帧（使用grab()和retrieve()提高性能）
                grabbed = self.camera.grab()
                if not grabbed:
                    retry_count += 1
                    if retry_count > max_retries:
                        self.get_logger().error(f"连续 {max_retries} 次无法抓取帧，尝试重新初始化摄像头")
                        self.camera.release()
                        self.camera = None
                        retry_count = 0
                        continue
                    else:
                        self.get_logger().warning(f"无法抓取帧 ({retry_count}/{max_retries})")
                        time.sleep(0.1)
                        continue
                
                # 重置重试计数器
                retry_count = 0
                
                # 解码帧
                ret, frame = self.camera.retrieve()
                if not ret:
                    self.get_logger().warning("无法解码帧")
                    time.sleep(0.1)
                    continue
                
                # 如果正在扫描，进行二维码检测
                if self.scanning:
                    # 检查扫描超时
                    if time.time() - self.scan_start_time > self.scan_timeout:
                        self.get_logger().warning("二维码扫描超时")
                        self.send_voice_feedback(b'\xAA\x55\xFF\x62\xFB')  # 发送"未检测到二维码"协议
                        self.scanning = False
                    
                    # 转换为灰度图像并检测二维码
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    barcodes = pyzbar.decode(gray)
                    
                    # 处理检测到的二维码
                    if barcodes:
                        for barcode in barcodes:
                            # 提取二维码数据
                            try:
                                barcode_data = barcode.data.decode("utf-8")
                                self.get_logger().info(f"扫描到二维码: {barcode_data}")
                                
                                # 发送识别成功反馈
                                self.send_voice_feedback(b'\xAA\x55\xFF\x5F\xFB')  # 发送"检测到二维码"协议
                                
                                # 在图像上绘制二维码位置和内容
                                self.draw_barcode(frame, barcode)
                                
                                # 扫描成功，停止扫描
                                self.scanning = False
                                
                                # 保存扫描结果图像
                                self.save_scan_result(frame, barcode_data)
                                
                            except Exception as e:
                                self.get_logger().error(f"二维码解码失败: {str(e)}")
                
                # 显示预览画面
                preview_frame = cv2.resize(frame, (0, 0), fx=self.preview_scale, fy=self.preview_scale)
                cv2.imshow(self.preview_window_name, preview_frame)
                
                # 检测键盘输入（ESC退出预览）
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    self.get_logger().info("用户手动退出预览")
                    self.preview_active = False
                    break
                
            except Exception as e:
                self.get_logger().error(f"预览线程出错: {str(e)}")
                time.sleep(0.5)
        
        self.get_logger().info("摄像头预览线程结束")
        if self.camera and self.camera.isOpened():
            self.camera.release()
        cv2.destroyWindow(self.preview_window_name)

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
        """显示二维码"""
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
            self.get_logger().info(f"二维码已显示: {self.qr_img_path}")
            
            # 发送语音反馈 (主动播报协议)
            self.send_voice_feedback(b'\xAA\x55\x00\x2F\xFB')  # 发送"请出示二维码"协议
        except Exception as e:
            self.get_logger().error(f"显示失败: {str(e)}")

    def close_display(self):
        """关闭当前显示"""
        if self.display_process and self.display_process.poll() is None:
            self.display_process.terminate()
            self.display_process.wait()
            self.get_logger().info("已关闭二维码显示")
            
            # 发送语音反馈 (主动播报协议)
            self.send_voice_feedback(b'\xAA\x55\x00\x2E\xFB')  # 发送"关闭二维码"协议
        self.display_process = None

    def scan_qr_code(self):
        """启动二维码扫描"""
        if self.scanning:
            self.get_logger().warning("扫描操作正在进行中，忽略新请求")
            return
            
        # 检查摄像头是否可用
        if self.camera is None or not self.camera.isOpened():
            self.get_logger().error("摄像头不可用，无法启动扫描")
            return
            
        # 设置扫描状态
        self.scanning = True
        self.scan_start_time = time.time()
        self.get_logger().info("开始扫描二维码...")
        
        # 发送开始扫描提示 (主动播报协议)
        self.send_voice_feedback(b'\xAA\x55\x00\x30\xFB')  # 发送"扫描二维码"协议

    def draw_barcode(self, frame, barcode):
        """在图像上绘制二维码框和文本"""
        (x, y, w, h) = barcode.rect
        # 绘制矩形框
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        # 显示二维码类型和数据（截短）
        try:
            barcode_type = barcode.type
            barcode_data = barcode.data.decode('utf-8')[:20] + '...'
        except:
            barcode_type = "UNKNOWN"
            barcode_data = "BINARY DATA"
            
        text = f"{barcode_type}: {barcode_data}"
        cv2.putText(frame, text, (x, y - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    def save_scan_result(self, frame, barcode_data):
        """保存扫描结果图像"""
        try:
            result_dir = "/home/sunrise/scan_results"
            os.makedirs(result_dir, exist_ok=True)
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            result_path = f"{result_dir}/qr_scan_{timestamp}.jpg"
            cv2.imwrite(result_path, frame)
            self.get_logger().info(f"扫描结果已保存至: {result_path}")
            
            # 同时保存二维码数据到文本文件
            txt_path = f"{result_dir}/qr_data_{timestamp}.txt"
            with open(txt_path, 'w') as f:
                f.write(barcode_data)
            self.get_logger().info(f"二维码数据已保存至: {txt_path}")
        except Exception as e:
            self.get_logger().error(f"保存扫描结果失败: {str(e)}")

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
        
        # 停止预览线程
        self.preview_active = False
        if self.preview_thread and self.preview_thread.is_alive():
            self.preview_thread.join(timeout=1.0)
        
        # 释放摄像头
        if hasattr(self, 'camera') and self.camera and self.camera.isOpened():
            self.camera.release()
        
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