cmake_minimum_required(VERSION 3.8)
project(vlm_social_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vsn_yolo_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SocialNavMsg.msg"
  DEPENDENCIES std_msgs
)

# ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/vision2social.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
