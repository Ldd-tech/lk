#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nmea_msgs.msg import Sentence
from std_msgs.msg import Header, String
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped, PoseStamped
from nav_msgs.msg import Path
import serial
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange
from collections import deque
from pyproj import Proj
from tf2_ros import TransformBroadcaster
import time
import math
import sys
import rclpy.parameter

class GPSDriver(Node):
    def __init__(self, context=None):
        super().__init__('gps_driver', context=context)
        
        # 参数列表 - 针对RDK X5优化
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyS1', ParameterDescriptor(description='串口设备路径')),  # RDK X5 UART1
                ('baudrate', 9600, ParameterDescriptor(
                    description='波特率',
                    integer_range=[IntegerRange(from_value=4800, to_value=115200, step=1)]
                )),
                ('timeout', 1.0, ParameterDescriptor(description='串口超时(秒)')),
                ('frame_id', 'utm', ParameterDescriptor(description='TF坐标系名称')),
                ('child_frame_id', 'gps_sensor', ParameterDescriptor(description='子坐标系名称')),
                ('marker_scale', 5.0, ParameterDescriptor(description='标记尺寸')),
                ('trail_length', 50, ParameterDescriptor(description='轨迹点数量')),
                ('utm_zone', 50, ParameterDescriptor(description='UTM区域号')),
                ('publish_rate', 1.0, ParameterDescriptor(description='发布频率(Hz)')),
                ('retry_interval', 2.0, ParameterDescriptor(description='重试间隔(秒)')),
                ('qos_depth', 10, ParameterDescriptor(description='QoS队列深度')),
                ('max_retries', 10, ParameterDescriptor(description='最大重试次数')),
                ('hdop_threshold', 5.0, ParameterDescriptor(description='最大允许HDOP值')),
                ('enable_cold_start_wait', False, ParameterDescriptor(description='是否启用冷启动等待')),
                ('cold_start_wait', 35.0, ParameterDescriptor(description='冷启动等待时间(秒)')),
                ('enable_config_commands', True, ParameterDescriptor(description='是否发送配置命令')),
                ('publish_path', True, ParameterDescriptor(description='是否发布路径')),
                ('path_frame', 'map', ParameterDescriptor(description='路径坐标系'))
            ])

        # 验证参数有效性
        self.validate_parameters()
        
        # 坐标转换器初始化
        self.utm_proj = Proj(proj='utm', zone=self.get_parameter('utm_zone').value, ellps='WGS84')
        self.tf_broadcaster = TransformBroadcaster(self, qos=self.get_parameter('qos_depth').value)
        
        # 初始化串口
        self.serial = None
        self.last_rmc_time = None
        self.init_serial()
        
        # 轨迹存储
        self.trail_points = deque(maxlen=self.get_parameter('trail_length').value)
        self.marker_id = 0
        self.current_utm_point = None
        self.antenna_status = "UNKNOWN"
        self.last_valid_fix_time = self.get_clock().now()
        self.path = Path()
        
        # QoS配置
        qos_profile = rclpy.qos.QoSProfile(
            depth=self.get_parameter('qos_depth').value,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )

        # 数据发布器
        self.nmea_pub = self.create_publisher(Sentence, '/nmea_sentence', qos_profile)
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', qos_profile)
        self.marker_pub = self.create_publisher(MarkerArray, '/gps/markers', qos_profile)
        self.status_pub = self.create_publisher(String, '/gps/status', qos_profile)
        
        # 新增路径发布器
        if self.get_parameter('publish_path').value:
            self.path_pub = self.create_publisher(Path, '/gps/path', qos_profile)
        
        # 主循环定时器
        timer_period = 1.0 / max(0.1, self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(timer_period, self.update_loop)
        
        # 冷启动等待（可选）
        if self.get_parameter('enable_cold_start_wait').value:
            cold_start_wait = self.get_parameter('cold_start_wait').value
            self.get_logger().info(f"等待冷启动完成 ({cold_start_wait}秒)...")
            time.sleep(cold_start_wait)
            self.get_logger().info("冷启动等待完成")

    def validate_parameters(self):
        """验证参数有效性并处理类型问题 - 修复波特率警告"""
        # 检查波特率参数类型 - 改进版本
        try:
            baudrate_param = self.get_parameter('baudrate')
            # 检查值是否有效，而不严格检查类型
            try:
                baudrate_value = int(baudrate_param.value)
            except (ValueError, TypeError):
                baudrate_value = 9600
                self.get_logger().error("波特率参数无效，使用默认值9600")
            
            # 仅在值不匹配时才更新参数
            if baudrate_param.value != baudrate_value:
                self.set_parameters([rclpy.parameter.Parameter(
                    'baudrate', 
                    rclpy.parameter.Parameter.Type.INTEGER, 
                    baudrate_value
                )])
                self.get_logger().info(f"设置波特率参数为: {baudrate_value}")

        except Exception as e:
            self.get_logger().error(f"参数验证错误: {str(e)}")

        # 验证其他参数
        params_to_check = [
            'trail_length', 'utm_zone', 'qos_depth', 'max_retries'
        ]
        
        for param in params_to_check:
            try:
                value = self.get_parameter(param).value
                if not isinstance(value, int) or value <= 0:
                    default = 50 if param == 'trail_length' else 50 if param == 'utm_zone' else 10
                    self.get_logger().warning(f"无效的{param}: {value}, 使用默认值{default}")
                    self.set_parameters([rclpy.parameter.Parameter(
                        param, 
                        rclpy.parameter.Parameter.Type.INTEGER, 
                        default
                    )])
            except Exception as e:
                self.get_logger().error(f"参数{param}验证失败: {str(e)}")

    def init_serial(self):
        """带重试机制的串口初始化 - 针对RDK X5优化"""
        max_retries = self.get_parameter('max_retries').value
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        self.get_logger().info(f"尝试连接串口: {port}, 波特率: {baudrate}")
        
        for i in range(max_retries):
            try:
                # RDK X5 UART1不需要流控制
                self.serial = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    timeout=self.get_parameter('timeout').value,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    rtscts=False,
                    dsrdtr=False
                )
                
                if self.serial.is_open:
                    self.get_logger().info(f"串口连接成功: {self.serial.name}")
                    self.serial.reset_input_buffer()
                    
                    # 发送配置命令确保GPS模块使用正确格式
                    if self.get_parameter('enable_config_commands').value:
                        try:
                            # 启用GGA和RMC语句
                            config_commands = [
                                b"$PUBX,40,GGA,0,1,0,0,0,0*5A\r\n",  # 启用GGA
                                b"$PUBX,40,RMC,0,1,0,0,0,0*47\r\n",  # 启用RMC
                                b"$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n",  # 禁用VTG
                                b"$PUBX,40,GSV,0,0,0,0,0,0*59\r\n",  # 禁用GSV
                                b"$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n",  # 禁用GLL
                                b"$PUBX,40,ZDA,0,0,0,0,0,0*44\r\n"   # 禁用ZDA
                            ]
                            
                            for cmd in config_commands:
                                self.serial.write(cmd)
                                time.sleep(0.05)
                                self.get_logger().debug(f"发送配置命令: {cmd.decode().strip()}")
                            
                            time.sleep(0.1)
                            self.serial.reset_input_buffer()
                        except Exception as e:
                            self.get_logger().warning(f"配置命令发送失败: {str(e)}")
                    
                    return True
                    
            except Exception as e:
                self.get_logger().error(f"串口连接失败(尝试 {i+1}/{max_retries}): {str(e)}")
                time.sleep(self.get_parameter('retry_interval').value)
        
        self.get_logger().critical(f"无法连接串口设备: {port}")
        return False

    def latlon_to_utm(self, lat, lon):
        """WGS84经纬度转UTM坐标"""
        try:
            if lat is None or lon is None:
                raise ValueError("无效的经纬度值")
                
            easting, northing = self.utm_proj(lon, lat)
            if math.isnan(easting) or math.isnan(northing):
                raise ValueError("坐标转换结果为NaN")
                
            return float(easting), float(northing)
        except Exception as e:
            self.get_logger().error(f"坐标转换失败: {str(e)}", throttle_duration_sec=30.0)
            return None, None

    def publish_tf(self, point):
        """发布坐标系变换"""
        if point is None:
            return
            
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.get_parameter('frame_id').value
            t.child_frame_id = self.get_parameter('child_frame_id').value
            t.transform.translation.x = point.x
            t.transform.translation.y = point.y
            t.transform.translation.z = point.z
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f"TF发布失败: {str(e)}", throttle_duration_sec=30.0)

    def parse_gga(self, line):
        """解析GGA语句 - 支持多种前缀"""
        # 支持GGA语句的多种前缀
        if not line.startswith(('$GPGGA', '$GNGGA', '$GAGGA', '$GLGGA', '$BDGGA')):
            return None, None
            
        try:
            data = line.split(',')
            if len(data) < 10 or not data[2] or not data[4]:
                return None, None

            # 检查定位质量
            fix_quality = int(data[6]) if data[6] else 0
            if fix_quality == 0:
                return None, None
                
            # 检查HDOP值
            hdop = float(data[8]) if data[8] else 99.0
            if hdop > self.get_parameter('hdop_threshold').value:
                self.get_logger().warning(f"高HDOP值: {hdop:.1f}, 定位精度差", throttle_duration_sec=30.0)
                
            # 解析经纬度
            lat = float(data[2][:2]) + float(data[2][2:])/60.0
            lon = float(data[4][:3]) + float(data[4][3:])/60.0
            if data[3] == 'S': lat *= -1
            if data[5] == 'W': lon *= -1
            
            # 构建NavSatFix消息
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.get_parameter('frame_id').value
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = float(data[9]) if data[9] else 0.0
            msg.status.status = fix_quality
            
            # 协方差矩阵
            cov = (hdop * 3.0)**2
            msg.position_covariance = [
                cov, 0.0, 0.0,
                0.0, cov, 0.0,
                0.0, 0.0, cov * 4
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            
            return msg, hdop
            
        except (ValueError, IndexError, TypeError) as e:
            self.get_logger().warning(f"GGA解析错误: {str(e)}", throttle_duration_sec=30.0)
            return None, None

    def parse_rmc(self, line):
        """解析RMC语句获取时间"""
        if not line.startswith(('$GPRMC', '$GNRMC', '$GLRMC', '$BDRMC')):
            return None
            
        try:
            data = line.split(',')
            if len(data) < 10 or not data[1] or not data[9]:
                return None
                
            # 提取UTC时间 (hhmmss.sss)
            utc_time = data[1]
            date_str = data[9]  # ddmmyy
            
            if len(utc_time) < 6 or len(date_str) < 6:
                return None
                
            try:
                # 转换日期和时间
                hours = int(utc_time[0:2])
                minutes = int(utc_time[2:4])
                seconds = float(utc_time[4:])
                
                day = int(date_str[0:2])
                month = int(date_str[2:4])
                year = 2000 + int(date_str[4:6])  # 假设是2000年之后的日期
                
                # 创建ROS时间对象 (简化处理，实际应使用UTC时间)
                # 注意：这里假设系统时钟已同步到正确时区
                now = self.get_clock().now().to_msg()
                return self.get_clock().now()
                
            except (ValueError, IndexError) as e:
                self.get_logger().warning(f"RMC时间解析错误: {str(e)}")
                return None
                
        except Exception as e:
            self.get_logger().warning(f"RMC解析错误: {str(e)}", throttle_duration_sec=30.0)
            return None

    def parse_antenna_status(self, line):
        """解析天线状态信息"""
        try:
            if "ANTENNA SHORT" in line:
                self.antenna_status = "SHORT"
                self.get_logger().error("天线短路！请检查天线连接")
            elif "ANTENNA OPEN" in line:
                self.antenna_status = "OPEN"
                self.get_logger().error("天线开路！请检查天线连接")
            elif "ANTENNA OK" in line:
                self.antenna_status = "OK"
                self.get_logger().info("天线状态正常", throttle_duration_sec=60.0)
            else:
                return
                
            # 发布状态消息
            status_msg = String()
            status_msg.data = self.antenna_status
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().warning(f"天线状态解析错误: {str(e)}")

    def create_markers(self, fix_msg):
        """生成可视化标记组"""
        markers = []
        
        try:
            # 当前位置标记
            if self.current_utm_point:
                pos_marker = Marker()
                pos_marker.header.stamp = fix_msg.header.stamp
                pos_marker.header.frame_id = fix_msg.header.frame_id
                pos_marker.ns = "gps_position"
                pos_marker.id = self.marker_id
                pos_marker.type = Marker.SPHERE
                pos_marker.action = Marker.ADD
                pos_marker.lifetime = rclpy.duration.Duration(seconds=5).to_msg()
                pos_marker.pose.position = self.current_utm_point
                
                scale = float(self.get_parameter('marker_scale').value)
                pos_marker.scale.x = scale
                pos_marker.scale.y = scale
                pos_marker.scale.z = scale * 0.5
                
                pos_marker.color.r = 1.0
                pos_marker.color.g = 0.4 if fix_msg.status.status == 1 else 1.0
                pos_marker.color.b = 0.0
                pos_marker.color.a = 0.9
                
                markers.append(pos_marker)
                self.marker_id = (self.marker_id + 1) % 1000
                
            # 轨迹标记
            if len(self.trail_points) > 1:
                trail_marker = Marker()
                trail_marker.header.stamp = fix_msg.header.stamp
                trail_marker.header.frame_id = fix_msg.header.frame_id
                trail_marker.ns = "gps_trail"
                trail_marker.id = self.marker_id
                trail_marker.type = Marker.LINE_STRIP
                trail_marker.action = Marker.ADD
                trail_marker.lifetime = rclpy.duration.Duration(seconds=10).to_msg()
                trail_marker.scale.x = 1.5
                trail_marker.color.r = 0.2
                trail_marker.color.g = 0.8
                trail_marker.color.b = 0.2
                trail_marker.color.a = 0.7
                trail_marker.points = list(self.trail_points)
                
                markers.append(trail_marker)
                self.marker_id = (self.marker_id + 1) % 1000

        except Exception as e:
            self.get_logger().error(f"标记生成错误: {str(e)}", throttle_duration_sec=30.0)
        
        return markers

    def update_loop(self):
        """主更新循环 - 针对ROS2 Humble优化"""
        # 检查数据有效性超时
        current_time = self.get_clock().now()
        timeout_duration = 15.0  # 15秒超时
        if (current_time - self.last_valid_fix_time).nanoseconds > timeout_duration * 1e9:
            self.get_logger().warning(f"GPS数据超时 ({timeout_duration}秒)，检查设备连接和天线位置")
            self.last_valid_fix_time = current_time
            
        # 串口连接检查
        if not self.serial or not self.serial.is_open:
            self.get_logger().warning("串口未连接，尝试重连...", throttle_duration_sec=10.0)
            if self.init_serial():
                self.get_logger().info("串口重连成功")
            else:
                return
            
        try:
            # 读取所有可用数据
            while self.serial.in_waiting > 0:
                try:
                    raw_line = self.serial.readline()
                    
                    if not raw_line:
                        continue
                        
                    # 尝试多种解码方式
                    line = None
                    for encoding in ['ascii', 'utf-8', 'latin-1']:
                        try:
                            line = raw_line.decode(encoding, errors='replace').strip()
                            break
                        except UnicodeDecodeError:
                            continue
                    
                    if line is None:
                        self.get_logger().debug("无法解码的串口数据", throttle_duration_sec=10.0)
                        continue
                    
                    # 处理天线状态消息
                    if line.startswith('$GPTXT'):
                        self.parse_antenna_status(line)
                        continue
                    
                    # 处理RMC语句
                    if rmc_time := self.parse_rmc(line):
                        self.last_rmc_time = rmc_time
                        self.get_logger().debug(f"更新RMC时间: {rmc_time}", throttle_duration_sec=5.0)
                    
                    # 发布原始NMEA消息
                    try:
                        nmea_msg = Sentence()
                        nmea_msg.sentence = line
                        self.nmea_pub.publish(nmea_msg)
                    except Exception as e:
                        self.get_logger().error(f"NMEA消息发布错误: {str(e)}", throttle_duration_sec=10.0)
                    
                    # 处理GGA语句
                    if line.startswith(('$GPGGA', '$GNGGA', '$GAGGA', '$GLGGA', '$BDGGA')):
                        result = self.parse_gga(line)
                        if result[0] is not None:
                            fix_msg, hdop_value = result
                            self.process_fix(fix_msg, hdop_value)
                            self.last_valid_fix_time = self.get_clock().now()
                except serial.SerialException as e:
                    self.get_logger().error(f"串口读取错误: {str(e)}")
                    if self.serial and self.serial.is_open:
                        try:
                            self.serial.close()
                        except:
                            pass
                    break
                except Exception as e:
                    self.get_logger().error(f"数据处理异常: {str(e)}", throttle_duration_sec=10.0)
                        
        except Exception as e:
            self.get_logger().error(f"数据读取异常: {str(e)}", throttle_duration_sec=10.0)

    def process_fix(self, msg, hdop):
        """处理定位数据，添加HDOP参数"""
        try:
            # 使用RMC时间更新消息时间戳（如果可用）
            if self.last_rmc_time:
                msg.header.stamp = self.last_rmc_time.to_msg()
            
            # 发布原始数据
            self.fix_pub.publish(msg)
            
            # 转换坐标
            easting, northing = self.latlon_to_utm(msg.latitude, msg.longitude)
            if easting is None or northing is None:
                return
                
            # 创建点对象
            point = Point()
            point.x = easting
            point.y = northing
            point.z = float(msg.altitude)
            
            # 存储当前点
            self.current_utm_point = point
            self.trail_points.append(point)
            
            # 发布TF变换
            self.publish_tf(point)
            
            # 发布可视化标记
            marker_array = MarkerArray()
            marker_array.markers = self.create_markers(msg)
            if marker_array.markers:
                self.marker_pub.publish(marker_array)
            
            # 发布路径
            if self.get_parameter('publish_path').value:
                self.update_path(point, msg.header.stamp)
            
            # 状态日志 - 使用传入的HDOP值
            self.get_logger().info(
                f"定位更新: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f} | "
                f"Alt={msg.altitude:.1f}m | HDOP={hdop:.1f} | "
                f"UTM: X={point.x:.1f}, Y={point.y:.1f}",
                throttle_duration_sec=1.0
            )
            
        except Exception as e:
            self.get_logger().error(f"数据处理错误: {str(e)}", throttle_duration_sec=10.0)
    
    def update_path(self, point, stamp):
        """更新并发布路径"""
        try:
            # 创建位姿
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.get_parameter('path_frame').value
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z
            pose.pose.orientation.w = 1.0
            
            # 更新路径
            self.path.header = pose.header
            self.path.poses.append(pose)
            
            # 限制路径长度
            max_path_length = self.get_parameter('trail_length').value * 2
            if len(self.path.poses) > max_path_length:
                self.path.poses = self.path.poses[-max_path_length:]
            
            # 发布路径
            self.path_pub.publish(self.path)
        except Exception as e:
            self.get_logger().error(f"路径更新失败: {str(e)}", throttle_duration_sec=10.0)

def main(args=None):
    context = rclpy.Context()
    rclpy.init(args=args, context=context)
    executor = None
    node = None
    
    try:
        node = GPSDriver(context=context)
        executor = MultiThreadedExecutor(num_threads=3, context=context)
        executor.add_node(node)
        
        # 安全spin循环
        while context.ok():
            executor.spin_once(timeout_sec=0.05)
            
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("接收到终止信号")
    except Exception as e:
        if node:
            node.get_logger().critical(f"致命错误: {str(e)}")
        else:
            print(f"节点初始化前错误: {str(e)}")
    finally:
        # 确保正确的关闭顺序
        try:
            if executor:
                executor.shutdown()
            if node:
                node.destroy_node()
        except:
            pass
        finally:
            if context.ok():
                context.shutdown()
            sys.exit(0)

if __name__ == '__main__':
    main()