cmake_minimum_required(VERSION 3.8)
project(cabot_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wfatal-errors)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(cabot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_util REQUIRED)
find_package(people_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cabot REQUIRED)


set(dependencies
  rclcpp
  rclcpp_components
  cabot
  cabot_msgs
  geometry_msgs
  nav_msgs
  people_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  nav2_util
)

add_library(cabot_handle_v2_lib SHARED
  src/cabot/cabot_handle_v2_node.cpp
  src/cabot/handle_v2.cpp
  src/cabot/event.cpp
)

add_library(cabot_handle_v3_lib SHARED
  src/cabot/cabot_handle_v3_node.cpp
  src/cabot/handle_v3.cpp
  src/cabot/event.cpp
)

ament_target_dependencies(cabot_handle_v2_lib
  ${dependencies}
)

ament_target_dependencies(cabot_handle_v3_lib
  ${dependencies}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(MAX_LINE_LENGTH 200)
  ament_copyright()
  ament_cpplint(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_flake8(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_lint_cmake()
  ament_uncrustify(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_xmllint()

  add_subdirectory(test)
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

add_library(stop_reasons_lib SHARED
  src/stop_reasoner.cpp
  src/stop_reasons_node.cpp
)

ament_target_dependencies(stop_reasons_lib
  ${dependencies}
)

rclcpp_components_register_node(
  stop_reasons_lib
  PLUGIN "CaBotUI::StopReasonsNode"
  EXECUTABLE stop_reasons_node
)

install(TARGETS
  stop_reasons_lib
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(PROGRAMS
  scripts/navcog_map.py
  scripts/cabot_keyboard.py
  scripts/cabot_ui_manager.py
  scripts/stop_reasons_node.py
  scripts/list_plugins.py
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_node(cabot_handle_v2_lib
  PLUGIN "CaBotHandleV2Node"
  EXECUTABLE cabot_handle_v2_node
)
rclcpp_components_register_node(cabot_handle_v3_lib
  PLUGIN "CaBotHandleV3Node"
  EXECUTABLE cabot_handle_v3_node
)

install(TARGETS
  cabot_handle_v2_lib
  cabot_handle_v3_lib
  DESTINATION lib
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz/)
install(DIRECTORY i18n/ DESTINATION share/${PROJECT_NAME}/i18n/)
install(DIRECTORY audio/ DESTINATION share/${PROJECT_NAME}/audio/)
install(DIRECTORY test/launch/ DESTINATION share/${PROJECT_NAME}/test/launch/)

ament_package()
