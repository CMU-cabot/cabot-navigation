cmake_minimum_required(VERSION 3.8)
project(social_nav_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(dwb_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(vlm_social_nav REQUIRED)

add_library(social_nav_plugin SHARED
  src/social_critic.cpp
)

target_include_directories(social_nav_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(social_nav_plugin
  rclcpp
  nav2_core
  nav2_costmap_2d
  dwb_core
  pluginlib
  vlm_social_nav
)

# target_link_libraries(social_nav_plugin
#   rclcpp::rclcpp
#   nav2_core::nav2_core
#   nav2_costmap_2d::nav2_costmap_2d
#   dwb_core::dwb_core
#   pluginlib::pluginlib
#   vlm_social_nav::vlm_social_nav
# )


pluginlib_export_plugin_description_file(dwb_core plugins_description.xml)

install(TARGETS social_nav_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(social_nav_plugin)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  nav2_core
  nav2_costmap_2d
  dwb_core
  pluginlib
  vlm_social_nav
)

ament_package()
