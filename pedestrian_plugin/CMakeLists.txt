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
find_package(pedestrian_plugin_msgs REQUIRED)
find_package(PythonLibs REQUIRED)


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
  rclcpp
  gazebo
  gazebo_ros
  people_msgs
  pedestrian_plugin_msgs
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
