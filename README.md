doubao.py 
python3 doubao.py运行代码。
火山引擎的豆包视觉大模型

gps_ws
显示gps位置信息。

guide_robot
ros2 launch guide_robot guide_robot.launch.py运行
此功能包融合了小车底盘、雷达避障、ai视觉检测通过语音模块播报出来。

voice_control
ros2 run voice_control voice_control_node启动语音控制节点
ros2 run voice_control speech_recognition_node启动语音识别节点。
运行两个指令之前要先启动机器人底盘。该包为融合机器人底盘启动文件。

nav2_waypoint_cycle
该包是机器人多点导航功能包。

voice_payment
ros2 launch voice_payment payment.launch.py
此包融合了语音控制摄像头识别付款码与语音控制出示付款码。

py_install
语音控制依赖库
搭建环境：
cd py_install
sudo python3 setup.py install
