cmake_minimum_required(VERSION 3.5)
project(four_wheeled_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vehicle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 编译插件
add_library(four_wheeled_vehicle_plugin SHARED
  src/four_wheeled_vehicle_plugin.cpp
)
target_include_directories(four_wheeled_vehicle_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(four_wheeled_vehicle_plugin
  ${GAZEBO_LIBRARIES}
)
ament_target_dependencies(four_wheeled_vehicle_plugin
  gazebo_ros
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  vehicle_msgs
)

include_directories(
  ${vehicle_msgs_INCLUDE_DIRS}
  ${rosidl_generator_c_INCLUDE_DIRS}
)

# 安装插件
install(TARGETS four_wheeled_vehicle_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# 安装模型、启动文件、世界文件和 rviz 配置
install(DIRECTORY models launch/ worlds/ rviz/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
