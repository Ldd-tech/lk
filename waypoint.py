#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_calibration.py - 航点标定工具
使用RViz的2D Nav Goal来标定各个位置的坐标
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json
import math
import os
import time

class WaypointCalibrationNode(Node):
    def __init__(self):
        super().__init__('waypoint_calibration_node')
        self.get_logger().info("航点标定节点已启动")
        
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.waypoints = {}
        self.current_waypoint_id = 1
        self.filename = "waypoints.json"
        
        # 预定义的特殊航点名称
        self.special_waypoints = {
            "家": 0x31,
            "超市": 0x32,
            "公园": 0x33,
            "学校": 0x34
        }
        
        # 加载已存在的航点
        self.load_existing_waypoints()

    def load_existing_waypoints(self):
        """加载已存在的航点文件"""
        if os.path.exists(self.filename):
            try:
                with open(self.filename, 'r', encoding='utf-8') as f:
                    self.waypoints = json.load(f)
                # 查找最大的ID号
                max_id = max([wp["id"] for wp in self.waypoints.values()], default=0)
                self.current_waypoint_id = max_id + 1
                self.get_logger().info(f"加载了 {len(self.waypoints)} 个航点，当前ID从 {self.current_waypoint_id} 开始")
            except Exception as e:
                self.get_logger().error(f"加载航点失败: {str(e)}")
                self.waypoints = {}

    def quaternion_to_yaw(self, orientation):
        """将四元数转换为偏航角"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def goal_callback(self, msg):
        """处理新的目标点"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.orientation)
        
        # 检查是否与现有航点过于接近
        for wp_id, waypoint in self.waypoints.items():
            distance = math.sqrt((x - waypoint["x"])**2 + (y - waypoint["y"])**2)
            if distance < 0.2:  # 20cm阈值
                self.get_logger().warning(f"位置过于接近航点 {wp_id} (距离: {distance:.3f}m)，使用现有航点")
                self.get_logger().info(f"航点 {wp_id}: {waypoint}")
                return
        
        # 提示用户输入航点名称
        self.get_logger().info("请输入航点名称（特殊名称：家/超市/公园/学校，或自定义名称）：")
        waypoint_name = input().strip()
        
        # 使用自定义名称或默认名称
        if not waypoint_name:
            waypoint_name = f"位置{self.current_waypoint_id}"
        
        # 检查是否是特殊航点
        command_id = self.special_waypoints.get(waypoint_name, self.current_waypoint_id)
        
        # 保存新航点
        self.waypoints[waypoint_name] = {
            "id": command_id,
            "x": round(x, 3),
            "y": round(y, 3),
            "yaw": round(yaw, 3),
            "name": waypoint_name
        }
        
        self.get_logger().info(f"标定 {waypoint_name}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} ({math.degrees(yaw):.1f}°)")
        self.current_waypoint_id += 1
        
        # 立即保存
        self.save_waypoints()
    
    def save_waypoints(self):
        """保存航点到文件"""
        if not self.waypoints:
            self.get_logger().info("没有标定的航点")
            return
        
        try:
            # 创建备份文件
            if os.path.exists(self.filename):
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                backup_name = f"{self.filename}.bak_{timestamp}"
                os.rename(self.filename, backup_name)
            
            with open(self.filename, 'w', encoding='utf-8') as f:
                json.dump(self.waypoints, f, ensure_ascii=False, indent=2)
            
            self.get_logger().info(f"航点已保存到 {self.filename}")
            self.log_waypoints()
                
        except Exception as e:
            self.get_logger().error(f"保存失败: {str(e)}")
    
    def log_waypoints(self):
        """记录所有航点信息"""
        self.get_logger().info("标定的航点:")
        for name, waypoint in self.waypoints.items():
            self.get_logger().info(
                f"  {name}: ID={waypoint['id']:02X}, 名称={waypoint['name']}, "
                f"x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']}"
            )

def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointCalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n正在保存航点...")
        node.save_waypoints()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()