cmake_minimum_required(VERSION 3.8)
project(test2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # 添加 sensor_msgs 依赖
find_package(Eigen3 REQUIRED)        # 添加 Eigen3 依赖

# 包含头文件路径
include_directories(
  include
  /usr/include/eigen3
)

# 添加可执行文件
add_executable(tf_broadcaster_node src/tf_broadcaster_node.cpp)
add_executable(ins_to_ecef_enu_node src/ins_to_ecef_enu_node.cpp)

# 链接依赖库
ament_target_dependencies(tf_broadcaster_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  Eigen3
)

ament_target_dependencies(ins_to_ecef_enu_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  Eigen3
)

# 安装目标
install(TARGETS
  tf_broadcaster_node
  ins_to_ecef_enu_node
  DESTINATION lib/${PROJECT_NAME}
)

# 测试部分
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_auto_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

