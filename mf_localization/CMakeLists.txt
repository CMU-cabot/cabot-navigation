cmake_minimum_required(VERSION 3.8)
project(mf_localization)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY configuration_files/ DESTINATION share/${PROJECT_NAME}/configuration_files)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(MAX_LINE_LENGTH 250)
  ament_copyright()
  ament_cpplint(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_flake8(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_lint_cmake()
  ament_uncrustify(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_xmllint()
endif()

ament_python_install_package(${PROJECT_NAME})

add_executable(multi_floor_topic_proxy
  src/multi_floor_topic_proxy.cpp
)

ament_target_dependencies(multi_floor_topic_proxy
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  yaml_cpp_vendor
)

add_executable(odometry_frame_converter_node
  src/odometry_frame_converter_node.cpp
)

ament_target_dependencies(odometry_frame_converter_node
  rclcpp
  nav_msgs
)

add_executable(tf_selector_node
  src/tf_selector_node.cpp
)

ament_target_dependencies(tf_selector_node
  rclcpp
  geometry_msgs
  tf2_ros
)


install(TARGETS
  multi_floor_topic_proxy
  odometry_frame_converter_node
  tf_selector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/fix_filter.py
  script/global_localizer_node.py
  script/imu_frame_renamer.py
  script/multi_floor_manager.py
  script/multi_floor_map_server.py
  script/multi_floor_wireless_rss_localizer.py
  script/tf2_beacons_listener.py
  script/tf2_listener.py
  script/tracked_pose_listener.py
  script/trajectory_based_interpolator.py
  script/trajectory_recorder.py
  script/trajectory_restarter.py
  script/str2str_node.py
  script/ublox_converter.py
  script/wireless_rss_localizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
