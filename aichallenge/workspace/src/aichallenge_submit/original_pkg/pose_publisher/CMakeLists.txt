cmake_minimum_required(VERSION 3.8)
project(pose_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pose_publisher_node src/pose_publisher_node.cpp)
ament_target_dependencies(pose_publisher_node rclcpp nav_msgs autoware_planning_msgs autoware_auto_planning_msgs geometry_msgs unique_identifier_msgs std_msgs)
install(TARGETS pose_publisher_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
