cmake_minimum_required(VERSION 3.8)
project(section_event_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(section_event_node src/section_event_node.cpp)
ament_target_dependencies(section_event_node rclcpp std_msgs)
install(TARGETS section_event_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()