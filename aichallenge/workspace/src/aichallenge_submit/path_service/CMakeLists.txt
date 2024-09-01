cmake_minimum_required(VERSION 3.8)
project(path_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # 必須: rosidl_default_generators の追加
find_package(nav_msgs REQUIRED)  # nav_msgs の依存関係の追加

# Generate the service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPath.srv"
  "srv/GetObstaclePath.srv"
  DEPENDENCIES nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
