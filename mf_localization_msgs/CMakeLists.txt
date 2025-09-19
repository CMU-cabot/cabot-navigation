cmake_minimum_required(VERSION 3.5)
project(mf_localization_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/StatusResponse.msg
  msg/MFDescriptor.msg
  msg/MFDescriptors.msg
  msg/MFGlobalPosition.msg
  msg/MFGlobalPosition2.msg
  msg/MFLocalPosition.msg
  msg/MFLocalizeStatus.msg
  msg/MFNavSAT.msg
)

set(srv_files
  srv/MFTrigger.srv
  srv/MFSetInt.srv
  srv/StopLocalization.srv
  srv/StartLocalization.srv
  srv/RestartLocalization.srv
  srv/FloorChange.srv
  srv/ConvertLocalToGlobal.srv
)

set(dependencies
  builtin_interfaces
  std_msgs
  geometry_msgs
)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
  ${dependencies}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  # find_package(ament_cmake_cpplint REQUIRED)
  # find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  # find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  # set(MAX_LINE_LENGTH 200)
  ament_copyright()
  # ament_cpplint(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  # ament_flake8(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_lint_cmake()
  # ament_uncrustify(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_xmllint()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
