cmake_minimum_required(VERSION 3.5)
project(pedestrian_plugin_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Plugin.msg
  msg/PluginParam.msg
  srv/PluginUpdate.srv
  DEPENDENCIES
)

ament_package()
