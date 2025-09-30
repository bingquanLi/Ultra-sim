cmake_minimum_required(VERSION 3.5)
project(joy_to_vehicle_cmd)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vehicle_msgs REQUIRED)

add_executable(joy_to_vehicle_cmd_node src/joy_to_vehicle_cmd_node.cpp)
ament_target_dependencies(joy_to_vehicle_cmd_node rclcpp sensor_msgs vehicle_msgs)

install(TARGETS
  joy_to_vehicle_cmd_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

