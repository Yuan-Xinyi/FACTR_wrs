cmake_minimum_required(VERSION 3.10)
project(franka_zmq_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(franka REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

add_executable(franka_zmq_controller_node src/controller_node.cpp)

target_include_directories(franka_zmq_controller_node PRIVATE
  ${ZMQ_INCLUDE_DIRS}
  ${franka_INCLUDE_DIRS}       # franka 包里导出的 include 路径
  include
)

target_link_libraries(franka_zmq_controller_node
  ${ZMQ_LIBRARIES}
  franka                       # 链接 franka 库
)

ament_target_dependencies(franka_zmq_controller_node rclcpp)

install(TARGETS franka_zmq_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
