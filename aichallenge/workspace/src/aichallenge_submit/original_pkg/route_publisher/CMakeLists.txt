cmake_minimum_required(VERSION 3.8)
project(route_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)

add_executable(route_publisher_node src/route_publisher_node.cpp)
ament_target_dependencies(route_publisher_node rclcpp  autoware_planning_msgs std_msgs)
install(TARGETS route_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
ament_package()
