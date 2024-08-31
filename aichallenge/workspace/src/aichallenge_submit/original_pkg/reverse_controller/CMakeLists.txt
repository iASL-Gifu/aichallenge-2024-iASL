cmake_minimum_required(VERSION 3.8)
project(reverse_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)

add_executable(reverse_controller_node src/reverse_control_node.cpp)

ament_target_dependencies(reverse_controller_node rclcpp  autoware_auto_control_msgs autoware_auto_vehicle_msgs)

install(TARGETS
  reverse_controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

