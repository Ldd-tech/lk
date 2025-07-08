cmake_minimum_required(VERSION 3.8)
project(ai_ros)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# 安装Python脚本作为可执行文件
install(PROGRAMS
  src/ai_ros/doubao.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python模块
install(DIRECTORY
  src/ai_ros
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()