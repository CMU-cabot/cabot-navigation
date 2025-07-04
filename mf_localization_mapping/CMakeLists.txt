cmake_minimum_required(VERSION 3.8)
project(mf_localization_mapping)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY configuration_files/ DESTINATION share/${PROJECT_NAME}/configuration_files)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

install(PROGRAMS
  script/check_topic_size.py
  script/compare_trajectory_and_gnss.py
  script/export_gnss_fix.py
  script/extract_floormap_info_from_yaml.py
  script/extract_node_options.py
  script/filter_rss_samples.py
  script/pbstream_edit.py
  script/timeout_thread.py
  script/topic_checker.py

  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
