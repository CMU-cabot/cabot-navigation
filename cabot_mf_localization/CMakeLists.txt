cmake_minimum_required(VERSION 3.8)
project(cabot_mf_localization)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  # find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  # find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(MAX_LINE_LENGTH 200)
  ament_copyright()
  # ament_cpplint(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_flake8(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_lint_cmake()
  # ament_uncrustify(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_xmllint()
endif()

ament_package()
