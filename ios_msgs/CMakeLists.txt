cmake_minimum_required(VERSION 3.8)
project(ios_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ARFrameWorldTrackingState.msg
  msg/ARFrameWorldTrackingStateStamped.msg
  msg/ARPointCloud.msg
  msg/ARPointCloudStamped.msg
  msg/CBPeripheral.msg
  msg/CBPeripheralStamped.msg
  msg/CLBeacon.msg
  msg/CLBeacons.msg
  msg/CMAccelerometerData.msg
  msg/CMCalibratedMagneticField.msg
  msg/CMDeviceMotion.msg
  msg/CMGyroData.msg
  msg/CMMagnetometerData.msg
  msg/NINearbyObject.msg
  msg/NINearbyObjects.msg
  msg/SimdFloat3.msg
  msg/SimdFloat4x4.msg
  msg/SimdFloat4x4Stamped.msg
  DEPENDENCIES std_msgs geometry_msgs
)

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

ament_package()
