#!/usr/bin/env python3
import cv2
import time
import queue
import threading
import json
import serial
import os
from base64 import b64encode
from openai import OpenAI
from ament_index_python.packages import get_package_share_directory

def load_config():
    try:
        # 获取功能包安装路径
        pkg_share_dir = get_package_share_directory('ai_ros')
        config_path = os.path.join(pkg_share_dir, 'config', 'config.json')
        
        with open('config.json', 'r', encoding='utf-8') as f:
            config = json.load(f)
            return config
    except FileNotFoundError:
        print("错误: 找不到config.json文件")
    except json.JSONDecodeError:
        print("错误: config.json格式不正确")
        exit(1)
    except Exception as e:
        print(f"错误: 读取配置文件时出错: {e}")
        exit(1)

class TrafficSpeech:
    def __init__(self, serial_port, baudrate=115200):
        try:
            self.ser = serial.Serial(serial_port, baudrate)
            if self.ser.isOpen():
                print(f"语音串口已打开! 端口: {serial_port}, 波特率: {baudrate}")
            else:
                print("语音串口打开失败!")
                self.ser = None
        except Exception as e:
            print(f"语音串口初始化失败: {e}")
            self.ser = None

    def __del__(self):
        if self.ser and self.ser.isOpen():
            self.ser.close()
            print("语音串口已关闭!")

    def send_traffic_light_command(self, light_type):
        """发送交通灯识别命令"""
        if not self.ser or not self.ser.isOpen():
            print("串口未打开，无法发送命令")
            return False
        
        # 根据识别结果发送对应的命令
        light_commands = {
            "黄灯": [0xAA, 0x55, 0xFF, 0x63, 0xFB],  # 88: 识别到黄灯
            "绿灯": [0xAA, 0x55, 0xFF, 0x64, 0xFB],  # 89: 识别到绿灯
            "蓝灯": [0xAA, 0x55, 0xFF, 0x65, 0xFB],  # 90: 识别到蓝灯
            "红灯": [0xAA, 0x55, 0xFF, 0x66, 0xFB],  # 91: 识别到红灯
        }
        
        if light_type in light_commands:
            try:
                cmd = light_commands[light_type]
                self.ser.write(bytearray(cmd))
                print(f"✓ 发送{light_type}命令: {[hex(x) for x in cmd]}")
                time.sleep(0.01)  
                self.ser.flushInput()
                return True
            except Exception as e:
                print(f"发送命令失败: {e}")
                return False
        else:
            print(f"未知的交通灯类型: {light_type}")
            return False

config = load_config()

client = OpenAI(
    api_key=config.get("api_key"),
    base_url=config.get("base_url", "https://ark.cn-beijing.volces.com/api/v3"),
)

speech_controller = TrafficSpeech(
    config.get("serial_port", "/dev/myspeech"),
    config.get("serial_baudrate", 115200)
)

MAX_SEND_TIMES = config.get("max_send_times", 10)
FRAME_SEND_RATE = config.get("frame_send_rate", 30)
MODEL_NAME = config.get("model", "doubao-1-5-vision-pro-32k-250115")

def is_person_detected(response_text: str) -> bool:
    keywords = ["红灯", "绿灯", "黄灯"]
    for kw in keywords:
        if kw in response_text:
            return True
    return False

def detect_traffic_light(response_text: str) -> str:
    if "红灯" in response_text:
        return "红灯"
    elif "绿灯" in response_text:
        return "绿灯"
    elif "黄灯" in response_text:
        return "黄灯"
    elif "蓝灯" in response_text:
        return "蓝灯"
    return None

def inference_thread_func(frame_queue: queue.Queue):
    send_count = 0
    
    while True:
        try:
            frame = frame_queue.get()
            
            if send_count >= MAX_SEND_TIMES:
                print(f"已达到最大发送次数 {MAX_SEND_TIMES}，停止发送")
                while not frame_queue.empty():
                    frame_queue.get_nowait()
                continue
            
            _, buffer = cv2.imencode('.jpg', frame)
            base64_image = b64encode(buffer).decode('utf-8')

            messages = [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": (
                                "请分析这个画面，每次都只用十五个字回答，"
                                "只关注红绿灯状态，只用："
                                "红灯,绿灯,黄灯，"
                                "如果没有看到红绿灯就说无红绿灯"。"
                                "如果看到红绿灯，请说明是红灯、绿灯还是黄灯。"
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

            response = client.chat.completions.create(
                model=MODEL_NAME,
                messages=messages
            )

            ai_reply = response.choices[0].message.content
            print(f"AI模型返回({send_count+1}/{MAX_SEND_TIMES}):", ai_reply)
            
            send_count += 1

            if is_person_detected(ai_reply):
                print("检测到有人出现，执行后续操作...")
            else:
                print("暂未检测到人，继续等待...")

            detected_light = detect_traffic_light(ai_reply)
            if detected_light:
                if speech_controller.send_traffic_light_command(detected_light):
                    print(f"识别到{detected_light}，已发送串口命令")
                else:
                    print(f"识别到{detected_light}，但串口命令发送失败")

        except Exception as e:
            print(f"推理时出现异常: {e}")
        time.sleep(0.5)  

def main():
    if not config.get("api_key"):
        print("错误: config.json中未找到api_key")
        return
    
    frame_queue = queue.Queue(maxsize=5)
    inference_thread = threading.Thread(target=inference_thread_func, args=(frame_queue,))
    inference_thread.daemon = True
    inference_thread.start()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    frame_counter = FRAME_SEND_RATE - 1
    actual_send_count = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法读取摄像头画面")
                break
                
            frame_counter += 1
            if frame_counter >= FRAME_SEND_RATE:
                frame_counter = 0

                if actual_send_count >= MAX_SEND_TIMES:
                    print("已达到最大发送次数，继续捕获帧但不发送")
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    continue

                if not frame_queue.full():
                    frame_queue.put(frame)
                    actual_send_count += 1
                    print(f"发送第 {actual_send_count} 帧到队列")
                else:

                    print("队列已满，等待处理...")
            else:
                pass

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
