#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import queue
import threading
import json
import os
from base64 import b64encode
from openai import OpenAI
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.bridge = CvBridge()
        
        # 初始化默认参数
        self.api_key = ""
        self.base_url = "https://ark.cn-beijing.volces.com/api/v3"
        self.model = "doubao-1-5-vision-pro-32k-250115"
        self.max_send_times = 100
        self.frame_send_rate = 30
        self.camera_topic = "/camera/image_raw"
        
        # 加载配置文件
        self.load_config()
        
        # 创建OpenAI客户端
        self.client = OpenAI(
            api_key=self.api_key,
            base_url=self.base_url
        )
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10)
        
        # 创建发布者
        self.voice_pub = self.create_publisher(String, '/voice_command', 10)
        
        # 创建帧队列和推理线程
        self.frame_queue = queue.Queue(maxsize=5)
        self.inference_thread = threading.Thread(target=self.inference_thread_func)
        self.inference_thread.daemon = True
        self.inference_thread.start()
        
        # 语音指令映射
        self.voice_commands = {
            "红灯": "red_light",
            "绿灯": "green_light",
            "黄灯": "yellow_light",
            "蓝灯": "blue_light",
            "车辆": "vehicle",
            "行人": "pedestrian",
            "斑马线": "crosswalk",
            "人行道": "sidewalk"
        }
        
        # 初始化状态变量
        self.last_detection = "无"
        self.detection_counter = 0
        self.frame_counter = 0
        
        self.get_logger().info("物体检测模块初始化完成")
        self.get_logger().info(f"使用模型: {self.model}")
        self.get_logger().info(f"摄像头主题: {self.camera_topic}")
        self.get_logger().info(f"最大发送次数: {self.max_send_times}")
        self.get_logger().info(f"帧发送频率: {self.frame_send_rate}")
        
        # 记录启动时间
        self.start_time = time.time()

    def load_config(self):
        try:
            # 获取功能包路径
            pkg_share_dir = get_package_share_directory('guide_robot')
            config_path = os.path.join(pkg_share_dir, 'config', 'config.json')
            
            self.get_logger().info(f"加载配置文件: {config_path}")
            
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
                
                # 读取配置参数
                self.api_key = config.get("api_key", "")
                self.base_url = config.get("base_url", "https://ark.cn-beijing.volces.com/api/v3")
                self.model = config.get("model", "doubao-1-5-vision-pro-32k-250115")
                self.max_send_times = config.get("max_send_times", 100)
                self.frame_send_rate = config.get("frame_send_rate", 30)
                self.camera_topic = config.get("camera_topic", "/camera/image_raw")
                
                self.get_logger().info("配置文件加载成功")
                
        except FileNotFoundError:
            self.get_logger().error("找不到配置文件，使用默认参数")
        except json.JSONDecodeError:
            self.get_logger().error("配置文件格式错误，使用默认参数")
        except Exception as e:
            self.get_logger().error(f"加载配置文件出错: {str(e)}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 调整图像大小以降低处理负载
            resized_image = cv2.resize(cv_image, (640, 480))
            
            # 更新帧计数器
            self.frame_counter += 1
            
            # 添加到队列
            if not self.frame_queue.full():
                self.frame_queue.put(resized_image)
                
        except Exception as e:
            self.get_logger().error(f"图像处理错误: {str(e)}")

    def inference_thread_func(self):
        send_count = 0
        frame_counter = 0
        
        while rclpy.ok():
            try:
                # 检查是否达到最大发送次数
                if send_count >= self.max_send_times:
                    self.get_logger().warn(f"达到最大发送次数 ({self.max_send_times}), 暂停推理")
                    time.sleep(5)
                    continue
                
                # 从队列获取帧
                frame = self.frame_queue.get(timeout=1.0)
                
                # 根据发送率控制处理频率
                frame_counter += 1
                if frame_counter < self.frame_send_rate:
                    continue
                frame_counter = 0
                
                # 记录发送时间
                self.get_logger().info(f"发送第 {send_count+1} 帧到队列")
                
                # 编码图像为base64
                _, buffer = cv2.imencode('.jpg', frame)
                base64_image = b64encode(buffer).decode('utf-8')
                
                # 构建消息
                messages = [
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": (
                                    "请分析交通场景，只用十五个字回答。"
                                    "识别内容包含：红绿灯状态（红灯/绿灯/黄灯/蓝灯），"
                                    "以及是否有车辆或行人，是否有斑马线或人行道。"
                                    "如果没有看到任何内容就说'无'。"
                                )
                            },
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{base64_image}"
                                }
                            }
                        ],
                    }
                ]
                
                # 发送推理请求
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    max_tokens=50
                )
                
                # 处理响应
                ai_reply = response.choices[0].message.content
                self.get_logger().info(f"AI模型返回({send_count+1}/{self.max_send_times}): {ai_reply}")
                
                # 解析检测结果
                detection_results = self.parse_detection_results(ai_reply)
                self.get_logger().info(f"场景解析: {detection_results}")
                
                # 更新检测状态
                self.last_detection = ai_reply
                send_count += 1
                
                # 处理检测结果
                self.process_detection(ai_reply)
                
            except queue.Empty:
                # 队列为空，等待下一帧
                pass
            except Exception as e:
                self.get_logger().error(f"推理异常: {str(e)}")
                time.sleep(1.0)  # 出错时等待1秒

    def parse_detection_results(self, ai_reply):
        """解析AI回复为结构化结果"""
        results = {
            "交通灯": "无",
            "斑马线": "无",
            "障碍物": "无",
            "车辆": "无",
            "行人": "无"
        }
        
        # 检查交通灯状态
        if "红灯" in ai_reply:
            results["交通灯"] = "红灯"
        elif "绿灯" in ai_reply:
            results["交通灯"] = "绿灯"
        elif "黄灯" in ai_reply:
            results["交通灯"] = "黄灯"
        elif "蓝灯" in ai_reply:
            results["交通灯"] = "蓝灯"
        
        # 检查其他对象
        if "斑马线" in ai_reply:
            results["斑马线"] = "有"
        if "行人" in ai_reply:
            results["行人"] = "有"
        if "车辆" in ai_reply:
            results["车辆"] = "有"
        if "障碍" in ai_reply:
            results["障碍物"] = "有"
        
        # 格式化为字符串
        return ", ".join([f"{k}={v}" for k, v in results.items()])

    def process_detection(self, response_text):
        # 检查所有可能的语音指令
        for keyword, command in self.voice_commands.items():
            if keyword in response_text:
                # 发送通用指令
                voice_msg = String()
                voice_msg.data = command
                self.voice_pub.publish(voice_msg)
                self.get_logger().info(f"检测到: {keyword}, 发送指令: {command}")
                
                # 如果是交通灯，额外发送中文指令
                if keyword in ["红灯", "绿灯", "黄灯", "蓝灯"]:
                    light_msg = String()
                    light_msg.data = keyword
                    self.voice_pub.publish(light_msg)
                    self.get_logger().info(f"发送交通灯指令: {keyword}")
                
                self.detection_counter += 1
                return
        
        # 如果没有检测到任何内容
        if "无" in response_text:
            self.get_logger().info("未检测到任何物体")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()