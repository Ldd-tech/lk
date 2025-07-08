#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import sys
import json
import math
import os
from collections import OrderedDict

# 添加Speech_Lib库的路径
sys.path.append('/home/sunrise/robot_ws/src/py_install/Speech_Lib')
from Speech_Lib import Speech

class SpeechNavigationNode(Node):
    def __init__(self):
        super().__init__('speech_navigation_node')
        self.get_logger().info("语音导航节点已启动")
        
        # 初始化语音识别模块
        try:
            self.speech = Speech("/dev/myspeech")
            self.get_logger().info("语音识别模块初始化成功")
        except Exception as e:
            self.get_logger().error(f"初始化失败: {str(e)}")
            raise

        self.voice_publisher = self.create_publisher(String, '/voice_words', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.timer = self.create_timer(0.1, self.check_speech)

        # 语音命令到航点名称的映射 (根据协议表)
        self.command_mapping = {
            0x31: "回家",
            0x32: "去超市",
            0x33: "去公园",
            0x34: "去学校",
            # 保留原有位置导航
            0x13: "位置1",
            0x14: "位置2", 
            0x15: "位置3",
            0x20: "位置4",  # 0x20 对应四号位
            0x21: "回到原点"
        }
        
        self.waypoints = self.load_waypoints()
        
        self.get_logger().info("语音导航节点初始化完成")
    
    def load_waypoints(self):
        """加载航点并确保唯一性"""
        waypoints = {}
        filename = "waypoints.json"
        
        if not os.path.exists(filename):
            self.get_logger().error(f"找不到 {filename} 文件，请先运行标定工具")
            return {}
        
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                raw_waypoints = json.load(f)
            
            # 使用OrderedDict保持插入顺序
            unique_waypoints = OrderedDict()
            
            for wp_name, wp_data in raw_waypoints.items():
                # 创建位置标识符
                location_id = f"{wp_data['x']:.2f},{wp_data['y']:.2f}"
                
                # 检查是否已存在相同位置
                if location_id in unique_waypoints:
                    self.get_logger().warning(
                        f"忽略重复航点 {wp_name} (位置: {location_id}), "
                        f"已存在 {unique_waypoints[location_id]['name']}"
                    )
                    continue
                
                # 添加到唯一航点集
                unique_waypoints[location_id] = {
                    "id": wp_data["id"],
                    "name": wp_data["name"],
                    "x": wp_data["x"],
                    "y": wp_data["y"],
                    "yaw": wp_data["yaw"]
                }
            
            # 转换为名称索引的字典
            result = {}
            for loc_id, wp in unique_waypoints.items():
                result[wp["name"]] = wp
            
            self.get_logger().info(f"成功加载 {len(result)} 个唯一航点")
            for name, waypoint in result.items():
                self.get_logger().info(
                    f"  {name}: ID={waypoint['id']:02X}, "
                    f"x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']}"
                )
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"加载航点失败: {str(e)}")
            return {}
    
    def quaternion_from_yaw(self, yaw):
        """从偏航角生成四元数"""
        half_yaw = yaw * 0.5
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(half_yaw),
            'w': math.cos(half_yaw)
        }
    
    def send_navigation_goal(self, waypoint_name):
        """发送导航目标"""
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f"找不到航点: {waypoint_name}")
            return False
        
        waypoint = self.waypoints[waypoint_name]
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = waypoint["x"]
        goal.pose.position.y = waypoint["y"]
        goal.pose.position.z = 0.0
        
        quaternion = self.quaternion_from_yaw(waypoint["yaw"])
        goal.pose.orientation.x = quaternion['x']
        goal.pose.orientation.y = quaternion['y']
        goal.pose.orientation.z = quaternion['z']
        goal.pose.orientation.w = quaternion['w']
        
        self.goal_publisher.publish(goal)
        self.get_logger().info(
            f"导航到 {waypoint_name}: "
            f"x={waypoint['x']:.3f}, y={waypoint['y']:.3f}, "
            f"yaw={waypoint['yaw']:.3f} ({math.degrees(waypoint['yaw']):.1f}°)"
        )
        return True
    
    def check_speech(self):
        """检查语音输入"""
        try:
            voice_data = self.speech.speech_read()
            
            if voice_data == 999:
                return
                
            self.get_logger().info(f"检测到语音数据: 0x{voice_data:02X}")
            
            if voice_data in self.command_mapping:
                command_name = self.command_mapping[voice_data]
                msg = String()
                msg.data = command_name
                self.voice_publisher.publish(msg)
                
                self.get_logger().info(f"执行命令: {command_name}")
                self.send_navigation_goal(command_name)
            else:
                self.get_logger().warning(f"未知命令值: 0x{voice_data:02X}")
                
        except Exception as e:
            self.get_logger().error(f"语音识别错误: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SpeechNavigationNode()
        node.get_logger().info("语音导航节点运行中，等待语音命令...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n收到键盘中断信号，正在关闭...")
    except Exception as e:
        print(f"节点异常: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()