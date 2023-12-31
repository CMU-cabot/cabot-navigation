cmake_minimum_required(VERSION 3.5)
project(cabot_navigation2)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(angles REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(people_msgs REQUIRED)
find_package(queue_msgs REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_critics REQUIRED)
find_package(diagnostic_updater REQUIRED)

# find dependencies for test
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_compression REQUIRED)
#find_package(OpenMP REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  ament_add_gtest_executable(test_util
    test/test_util.cpp
    plugins/util.cpp
    plugins/cabot_planner_util.cpp
    plugins/navcog_path_util.cpp
  )
  ament_target_dependencies(test_util rclcpp nav2_costmap_2d OpenCV)
  add_ros_test(test/test_util.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()

include_directories(
  include
#  ${OPENMP_INCLUDES}
  ${OpenCV_INCLUDE_DIRS}
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" )
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

if (NOT DEBUG) 

set(dependencies
  geometry_msgs
  rclcpp
  nav2_lifecycle_manager
  nav2_costmap_2d
  nav2_core
  angles
  nav_msgs
  pluginlib
  tf2
  people_msgs
  queue_msgs
  sensor_msgs
  dwb_core
  dwb_critics
  diagnostic_updater
  yaml_cpp_vendor
#  OpenMP
  )

### dwb critics
add_library(cabot_critics SHARED
  plugins/pose_align.cpp
  plugins/velocity_obstacle.cpp
  plugins/path_forward_prefer.cpp
  )

target_compile_definitions(cabot_critics PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(cabot_critics
  ${dependencies}
  )

### goal checker
add_library(cabot_goals SHARED
  plugins/cabot_simple_goal_checker.cpp
  plugins/util.cpp
  )

target_compile_definitions(cabot_goals PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(cabot_goals
  ${dependencies}
  )

### costmap layers
add_library(cabot_layers SHARED
  plugins/navcog_path_layer.cpp
  plugins/people_obstacle_layer.cpp
  plugins/navcog_path_util.cpp
  )

target_compile_definitions(cabot_layers PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(cabot_layers
  ${dependencies}
  )

### planner plugin
add_library(cabot_planners SHARED
  plugins/navcog_path_planner.cpp
  plugins/cabot_planner.cpp
  plugins/cabot_planner_param.cpp
  plugins/cabot_planner_util.cpp
  plugins/navcog_path_util.cpp
  )

target_compile_definitions(cabot_planners PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(cabot_planners
  ${dependencies}
  )

target_link_libraries(cabot_planners
  ${OpenCV_LIBS}
#  ${OPENMP_LIBRARIES}
)

### scan multiplier
add_executable(cabot_scan
  src/cabot_scan.cpp
  )

ament_target_dependencies(cabot_scan
  ${dependencies}
  )

### cabot lifecycle manager
add_executable(cabot_lifecycle_manager
  src/cabot_lifecycle_manager.cpp
  )
ament_target_dependencies(cabot_lifecycle_manager
  ${dependencies}
  )

install(TARGETS
  cabot_critics
  cabot_goals
  cabot_layers
  cabot_planners
  cabot_scan
  cabot_lifecycle_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  critics_plugins.xml
  goal_plugins.xml
  layers_plugins.xml
  planners_plugins.xml
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY test/ DESTINATION share/${PROJECT_NAME}/test/)

install(PROGRAMS
  test/run_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  cabot_critics
  cabot_goals
  cabot_layers
  cabot_planners)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(dwb_core critics_plugins.xml)
pluginlib_export_plugin_description_file(nav2_core goal_plugins.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d layers_plugins.xml)
pluginlib_export_plugin_description_file(nav2_core planners_plugins.xml)

endif()

if (DEBUG)
### test

set(dependencies_test
  diagnostic_updater
  geometry_msgs
  sensor_msgs
  people_msgs
  queue_msgs
  rclcpp
  nav2_lifecycle_manager
  nav2_navfn_planner
  nav2_costmap_2d
  nav2_map_server
  yaml_cpp_vendor
  tf2
  rosbag2
  rosbag2_cpp
  rosbag2_transport
  rosbag2_storage
  rosbag2_compression
#  OpenMP
)

find_package(Boost COMPONENTS system filesystem REQUIRED)
add_executable(cabot_planner_test
  test/cabot_planner_test.cpp
  plugins/cabot_planner.cpp
  plugins/cabot_planner_param.cpp
  plugins/cabot_planner_util.cpp
  plugins/navcog_path_util.cpp
)
ament_target_dependencies(cabot_planner_test
  ${dependencies_test}
)
### cabot lifecycle manager
add_executable(cabot_lifecycle_manager
  src/cabot_lifecycle_manager.cpp
)
ament_target_dependencies(cabot_lifecycle_manager
  ${dependencies_test}
)

target_include_directories(cabot_planner_test PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(cabot_planner_test
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${OpenCV_LIBS}
#  ${OPENMP_LIBRARIES}
)

install(TARGETS
  cabot_planner_test
  cabot_lifecycle_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY test/ DESTINATION share/${PROJECT_NAME}/test/)

endif()

ament_package()
