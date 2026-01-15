cmake_minimum_required(VERSION 3.8)
project(nav2_local_to_global_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 2. Create the Library
add_library(${PROJECT_NAME} SHARED
  src/local_to_global_layer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. Link Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_costmap_2d
  nav_msgs
  pluginlib
  tf2_ros
  tf2_geometry_msgs
)

# 4. Register the Plugin (CRITICAL STEP)
# This macro ensures the plugin is indexed by ROS 2
pluginlib_export_plugin_description_file(nav2_costmap_2d plugin.xml)

# 5. Install Rules
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# 6. Finalize Package
ament_package()