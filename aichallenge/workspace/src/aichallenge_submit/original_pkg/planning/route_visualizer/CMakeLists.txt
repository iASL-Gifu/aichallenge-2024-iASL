cmake_minimum_required(VERSION 3.8)
project(route_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


add_executable(route_visualizer_node src/route_visualizer_node.cpp)
ament_target_dependencies(route_visualizer_node rclcpp  autoware_planning_msgs visualization_msgs)
install(TARGETS route_visualizer_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()