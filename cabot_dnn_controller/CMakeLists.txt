cmake_minimum_required(VERSION 3.5)
project(cabot_dnn_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_library(NVINFER_LIB nvinfer)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin)
find_package(yaml-cpp REQUIRED)

add_library(dnn_controllers SHARED
  plugins/dagger_controller.cpp
  plugins/dnn_controller.cpp
)

target_include_directories(dnn_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dnn_controllers
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  people_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
)

target_link_libraries(dnn_controllers
  CUDA::cudart
  ${NVINFER_LIB}
  ${NVINFER_PLUGIN_LIB}
  ${OpenCV_LIBS}
  yaml-cpp
)

add_executable(test_dnn_controller_model
  src/test_dnn_controller_model.cpp
)

target_include_directories(test_dnn_controller_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(test_dnn_controller_model
  CUDA::cudart
  ${NVINFER_LIB}
  ${NVINFER_PLUGIN_LIB}
  yaml-cpp
)

install(TARGETS dnn_controllers test_dnn_controller_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(nav2_core controllers_plugin.xml)

install(FILES controllers_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(dnn_controllers)

ament_package()
