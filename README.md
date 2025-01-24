![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)

# CaBot Navigation Packages

|Package|Description|
|---|---|
|[cabot](./cabot)|This package includes cabot basic functions|
|[cabot_bt](./cabot_bt)|cabot behavior trees (BT), BT plugins, and some utilities|
|[cabot_description](./cabot_description)|robot URDF description|
|[cabot_gazebo](./cabot_gazebo)|robot launch files for gazebo environment, counter part of cabot package|
|[cabot_mf_localization](./cabot_mf_localization)|launch file and script for launching multi floor localization using RF signals (WiFi/BLE) and cartographer|
|[cabot_msgs](./cabot_msgs)|cabot message definition|
|[cabot_navigation2](./cabot_navigation2)|cabot core navigation logic using Nav2|
|[cabot_sites](./cabot_sites)|place cabot site packages|
|[cabot_ui](./cabot_ui)|user interface related code and i18n files|
|[mf_localization](./mf_localization)|multi floor localization function.|
|[mf_localization_gazebo](./mf_localization_gazebo)|gazebo utility to test multi floor localization|
|[mf_localization_mapping](./mf_localization_mapping)|mapping function for multi floor localization.|
|[mf_localization_msgs](./mf_localization_msgs)|message difinitions for multi floor localization|
|[mf_localization_rviz](./mf_localization_rviz)|rviz plugins for multi floor localization control (floor up/down, restart localization)|
|[queue_msgs](./queue_msgs)|msgs for queue packages|
|[queue_people_py](./queue_people_py)|publish queue message to control robot in queue|
|[queue_utils_py](./queue_utils_py)|utilities for queue|


## Test

- Tests in thie repo is solely for navigation and localization not user interfaces such as buttons, vibration pattern, speech etc.
- Such UI test should be done in the [cabot](https://github.com/cmu-cabot/cabot) repository

### Preparation

- run the script to download dependencies

```
./setup-dependency.sh
./setup-sample-site.sh
```

- run the script to build image and workspaces

```
./build-docker.sh -p -i -w              # run prebuild, image build, and workspace build
or
./build-docker.sh -p -i -w -d           # run prebuild, image build, and workspace debug build (symlink-install)
```

### Run simulator and run test cases

- set .env file (example)

```
CABOT_MODEL=cabot2-gt1
CABOT_SITE=cabot_site_cmu_3d
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

- run simulator

```
./launch.sh -s                 # simulation, yes to launch a map server
```

- run test cases

```
./launch.sh -s -t              # simulation, yes to launch a map server, run test
./launch.sh -s -t -H           # simulation, yes to launch a map server, run test, headless
```

### .env file

- **Required settings**
  ```
  CABOT_MODEL          # robot model (default=) to determine which launch/urdf to use
  CABOT_SITE           # package name for cabot site (default=)
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # need to use cyclone dds due to performance issue
  ```
- Optional settings
  ```
  CABOT_LANG           # cabot language (default=en)
  CABOT_OFFSET         # offset size (default=0.25)
  CABOT_FOOTPRINT_RADIUS # robot radius size (default=0.45)
  CABOT_INIT_SPEED     # specify maximum robot speed at startup, leave empty to restore the last speed
  CABOT_MAX_SPEED      # you can change max_speed only when CABOT_MODEL is cabot2-gtm (defalt=1.0)
  CABOT_USE_GNSS       # to use GNSS fix for localization (default=0)
  CABOT_ANNOUNCE_NO_TOUCH # announce when the reason robot is stopped is NO_TOUCH(default=false)
  CABOT_SIDE           # left: user stands on the right, right: user stands on the left
  CYCLONEDDS_NETWORK_INTERFACE_NAME # to specify network interface name for Cyclone DDS
  ```
- Options for image description (hold down right button for 1-3 seconds)
  ```
  CABOT_IMAGE_DESCRIPTION_SERVER   # image description server (default=http://localhost:8000)
  CABOT_IMAGE_DESCRIPTION_ENABLED  # enable image description (default=false)
  CABOT_IMAGE_DESCRIPTION_MODE     # surround and/or stop-reason separated by comma (default=surround), or stop-reason-data-collect
  CABOT_IMAGE_DESCRIPTION_ROTATE_LEFT   # rotate left image (default=false)
  CABOT_IMAGE_DESCRIPTION_ROTATE_FRONT  # rotate front image (default=false)
  CABOT_IMAGE_DESCRIPTION_ROTATE_RIGHT  # rotate right image (default=false)
  ```
- Options for debug/test
  ```
  CABOT_GAMEPAD              # (default=gamepad) gamepad type for remote controll (ex. PS4 controller)
                                               pro (Nintendo Switch Pro controller)
  CABOT_USE_HANDLE_SIMULATOR # to use handle simulator (default=0)
  CABOT_REMOTE_USE_KEYBOARD  # to use teleop twist keayboard in the remote control mode (default=false)
  CABOT_REMOTE_USE_IMU       # to use imu in the remote control mode (default=false)
  CABOT_SHOW_GAZEBO_CLIENT   # show gazebo client (default=0)
  CABOT_SHOW_ROS1_RVIZ       # show ROS1 rviz (default=0)
  CABOT_SHOW_ROS2_RVIZ       # show ROS2 rviz (default=1)
  CABOT_SHOW_ROS2_LOCAL_RVIZ # show ROS2 local navigation rviz (default=0)
  CABOT_SHOW_LOC_RVIZ        # show ROS1 localization rviz (default=1)
  CABOT_SHOW_PEOPLE_RVIZ     # show ROS1 people rviz (default=0)
  CABOT_SHOW_ROBOT_MONITOR   # show robot monitor (default=1)
  CABOT_RECORD_ROSBAG2       # record BT log, controller critics evalation into rosbag2 (default=1)
  CABOT_USE_ROBOT_TTS        # use TTS service '/speak_robot' to let PC speaker speak (default=0)
                             # this function is not used now, but maybe used in some scenario
  TEXT_TO_SPEECH_APIKEY      # IBM Cloud Text to Speech Service's API key and URL
  TEXT_TO_SPEECH_URL         # these two variables are required if CABOT_USE_ROBOT_TTS is 1
  ```
- Options for simulation
  ```
  CABOT_INITX          # initial robot position x for gazebo (default=0)
  CABOT_INITY          # initial robot position y for gazebo (default=0)
  CABOT_INITZ          # initial robot position z for gazebo (default=0)
  CABOT_INITA          # initial robot angle (degree) for gazebo (default=0)
  ```
- Others
  ```
  ## The following will be managed by docker-compose files
  ## be careful to set these variables in your .env file
  ##
  CABOT_GAZEBO             # 1: gazebo 0: real robot
  CABOT_TOUCH_ENABLED      # true: enabled - false: disabled
                             disabled for gazebo and enabled for real robot in docker-compose file
  CABOT_USE_REALSENSE      # to use realsense camera (default=0)
                             disabled for gazebo and enabled for real robot in docker-compose file
  CABOT_PRESSURE_AVAILABLE # to use pressure sensor (default=0)
                             disabled for gazebo and enabled for real robot in docker-compose file
  ```
- DDS related
  ```
  NET_CORE_BUFFER_SIZE_IN_MB   # default 64, small buffer size can cause data congestion and degrade performance especially velodyne_points and localization
  ```

# License

[MIT License](LICENSE)


---
The following files/folder are under Apache-2.0 License

- cabot_navigation/launch/cartographer_mapping.launch
