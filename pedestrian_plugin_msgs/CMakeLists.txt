cmake_minimum_required(VERSION 3.5)
project(pedestrian_plugin_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Agent.msg
  msg/Agents.msg
  msg/Collision.msg
  msg/Obstacle.msg
  msg/ObstacleCollision.msg
  msg/Metric.msg
  msg/Plugin.msg
  msg/PluginParam.msg
  srv/PluginUpdate.srv
  DEPENDENCIES geometry_msgs people_msgs std_msgs
)

ament_package()
