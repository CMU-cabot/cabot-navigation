cmake_minimum_required(VERSION 3.8)
project(cabot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wfatal-errors)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  rcutils
  diagnostic_updater
  geometry_msgs
  nav_msgs
  people_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

find_package(rclcpp REQUIRED)

include_directories(
  include
  ${tf2_INCLUDE_DIRS}
)

add_library(speed_visualize SHARED
  src/cabot/speed_visualize_node.cpp
)
ament_target_dependencies(speed_visualize
  ${dependencies}
)
rclcpp_components_register_node(
  speed_visualize
  PLUGIN "CaBot::SpeedVisualizeNode"
  EXECUTABLE speed_visualize_node
)

add_library(cabot SHARED
  src/safety/util.cpp
)

add_library(safety_nodes SHARED
  src/safety/clearing_tf_node.cpp
  src/safety/lidar_speed_control_node.cpp
  src/safety/odom_adapter_node.cpp
  src/safety/people_speed_control_node.cpp
  src/safety/speed_control_node.cpp
  src/safety/tf_speed_control_node.cpp
  src/safety/touch_speed_control_node.cpp
  src/safety/util.cpp
  src/safety/wheelie_speed_control_node.cpp
)

ament_target_dependencies(cabot
  ${dependencies}
)

ament_target_dependencies(safety_nodes
  ${dependencies}
)

rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::ClearingTFNode"
  EXECUTABLE clearing_tf_node
)
rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::LiDARSpeedControlNode"
  EXECUTABLE lidar_speed_control_node
)
rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::OdomAdapterNode"
  EXECUTABLE odom_adapter_node
  EXECUTOR "MultiThreadedExecutor"
)
rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::PeopleSpeedControlNode"
  EXECUTABLE people_speed_control_node
)
rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::SpeedControlNode"
  EXECUTABLE speed_control_node
)
rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::TFSpeedControlNode"
  EXECUTABLE tf_speed_control_node
)
rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::TouchSpeedControlNode"
  EXECUTABLE touch_speed_control_node
)
rclcpp_components_register_node(
  safety_nodes
  PLUGIN "CaBotSafety::WheelieControlNode"
  EXECUTABLE wheelie_speed_control_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cpplint(MAX_LINE_LENGTH 150)
  ament_flake8(MAX_LINE_LENGTH 150)
  ament_lint_cmake()
  ament_uncrustify(MAX_LINE_LENGTH 150)
  ament_xmllint()
endif()

# Install shared libraries
install(TARGETS
  speed_visualize
  safety_nodes
  cabot
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories("include/")
ament_export_libraries(cabot)
ament_export_dependencies(${dependencies})

ament_package()
