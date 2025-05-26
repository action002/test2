cmake_minimum_required(VERSION 3.5)
project(test2)

# 查找所需的 ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# 查找头文件目录
include_directories(${EIGEN3_INCLUDE_DIR})  # 可以去掉

include_directories(include)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

# 添加源文件
add_executable(tf_broadcaster_node src/tf_broadcaster_node.cpp)
add_executable(ins_to_ecef_enu_node src/ins_to_ecef_enu_node.cpp)

# 将可执行文件链接到所需的库
ament_target_dependencies(tf_broadcaster_node
  rclcpp
  geometry_msgs
  tf2_ros
  Eigen3 
)

ament_target_dependencies(ins_to_ecef_enu_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  Eigen3
)

# 安装可执行文件
install(TARGETS
  tf_broadcaster_node
  ins_to_ecef_enu_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 生成编译文件
ament_package()

