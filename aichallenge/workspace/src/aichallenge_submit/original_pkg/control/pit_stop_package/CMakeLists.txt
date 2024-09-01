cmake_minimum_required(VERSION 3.8)
project(pit_stop_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tier4_planning_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(pit_stop_node src/pit_stop_node.cpp)

ament_target_dependencies(pit_stop_node rclcpp tier4_planning_msgs nav_msgs autoware_auto_vehicle_msgs)

install(TARGETS
  pit_stop_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
