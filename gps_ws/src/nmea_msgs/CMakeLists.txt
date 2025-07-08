cmake_minimum_required(VERSION 3.8)
project(nmea_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖项配置
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息接口（Python 生成器默认启用）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sentence.msg"
  "msg/Gpgga.msg"
  DEPENDENCIES std_msgs
)

# 手动安装Python消息文件到正确路径
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/nmea_msgs"
  DESTINATION "lib/python3.10/site-packages"
)

# 导出依赖项
ament_export_dependencies(rosidl_default_runtime)

# 测试配置（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
