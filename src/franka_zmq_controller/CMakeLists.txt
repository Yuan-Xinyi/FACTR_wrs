cmake_minimum_required(VERSION 3.8)
project(franka_zmq_controller)

# 编译器警告（可选）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找 ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Franka REQUIRED)

# ZeroMQ
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# 添加可执行文件
add_executable(franka_zmq_controller_node
  src/controller_node.cpp
)

# 添加 C++17 标准（注意：要在 add_executable 之后）
target_compile_features(franka_zmq_controller_node PUBLIC cxx_std_17)

# 包含头文件路径
target_include_directories(franka_zmq_controller_node PRIVATE
  ${ZMQ_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  include
)

# 链接库
target_link_libraries(franka_zmq_controller_node
  ${ZMQ_LIBRARIES}
  franka
)

# ROS 依赖
ament_target_dependencies(franka_zmq_controller_node
  rclcpp
  sensor_msgs
  std_msgs
)

# 安装
install(TARGETS franka_zmq_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
