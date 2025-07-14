cmake_minimum_required(VERSION 3.5)
project(cabot_gazebo)

# find dependencies
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

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY params/ DESTINATION share/${PROJECT_NAME}/params)
install(DIRECTORY script/ DESTINATION lib/${PROJECT_NAME})
install(FILES
  plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  script/wireless_rss_simulator_node.py
  script/wireless_sample_simulator_node.py
  script/check_gazebo_ready.py
  script/pressure_simulator_node.py
  script/gps_converter.py
  script/cabot_handle_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
