cmake_minimum_required(VERSION 3.8)
project(object_event_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)

add_executable(object_event_node src/object_event_node.cpp)

ament_target_dependencies(object_event_node rclcpp std_msgs autoware_auto_perception_msgs)

install(TARGETS
  object_event_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

