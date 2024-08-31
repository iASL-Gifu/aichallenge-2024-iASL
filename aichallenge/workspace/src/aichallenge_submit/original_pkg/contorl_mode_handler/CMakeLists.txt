cmake_minimum_required(VERSION 3.8)
project(contorl_mode_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(control_event_node src/control_event_node.cpp)

ament_target_dependencies(control_event_node rclcpp  autoware_auto_vehicle_msgs std_msgs)

install(TARGETS
  control_event_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
