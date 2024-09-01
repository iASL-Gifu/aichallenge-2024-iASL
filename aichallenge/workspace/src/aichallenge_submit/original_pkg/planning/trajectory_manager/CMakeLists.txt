cmake_minimum_required(VERSION 3.8)
project(trajectory_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)

add_executable(trajectory_manage_node src/trajectory_manage_node.cpp)

ament_target_dependencies(trajectory_manage_node rclcpp std_msgs autoware_auto_planning_msgs autoware_planning_msgs)

install(TARGETS
  trajectory_manage_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()