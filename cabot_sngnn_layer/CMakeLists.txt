cmake_minimum_required(VERSION 3.8)
project(cabot_sngnn_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_library(cabot_sngnn_layer SHARED
  src/cabot_sngnn_layer.cpp
)
target_include_directories(cabot_sngnn_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cabot_sngnn_layer
  rclcpp
  nav2_costmap_2d
  nav_msgs
  pluginlib
  tf2_geometry_msgs
  tf2
)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(TARGETS cabot_sngnn_layer
  EXPORT export_cabot_sngnn_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(cabot_sngnn_layer)
ament_export_include_directories(include)
ament_package()
