cmake_minimum_required(VERSION 3.8)
project(wheelie_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_executable(wheelie_control_node src/wheelie_control_node.cpp)

ament_target_dependencies(wheelie_control_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  wheelie_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories("include/")
ament_export_libraries(wheelie_control_node)
ament_export_dependencies(${dependencies})

ament_package()
