cmake_minimum_required(VERSION 3.8)
project(pedestrian_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wfatal-errors)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(people_msgs REQUIRED)
find_package(PythonLibs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

#link_directories(
#  ${catkin_LIBRARY_DIRS}
#  ${GAZEBO_LIBRARY_DIRS}
#)

add_library(${PROJECT_NAME} SHARED
  src/pedestrian_plugin.cpp
  src/pedestrian_plugin_manager.cpp
  src/python_module_loader.cpp
  src/walking_pose.cpp
)

#target_include_directories(pedestrian_plugin PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "gazebo"
  "gazebo_ros"
  "people_msgs"
)
ament_export_libraries(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

# Install Python modules
ament_python_install_package("pedestrian")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
