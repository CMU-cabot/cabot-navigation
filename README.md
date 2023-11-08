![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)

# CaBot Navigation Packages

|Package|Description|
|---|---|
|[cabot](../cabot)|This package includes cabot basic functions|
|[cabot_bt](../cabot_bt)|cabot behavior trees (BT), BT plugins, and some utilities|
|[cabot_description](../cabot_description)|robot URDF description|
|[cabot_gazebo](../cabot_gazebo)|robot launch files for gazebo environment, counter part of cabot package|
|[cabot_mf_localization](../cabot_mf_localization)|launch file and script for launching multi floor localization using RF signals (WiFi/BLE) and cartographer|
|[cabot_msgs](../cabot_msgs)|cabot message definition|
|[cabot_navigation2](../cabot_navigation2)|cabot core navigation logic using Nav2|
|[cabot_sites](../cabot_sites)|place cabot site packages|
|[cabot_ui](../cabot_ui)|user interface related code and i18n files|
|[mf_localization](../mf_localization)|multi floor localization function.|
|[mf_localization_gazebo](../mf_localization_gazebo)|gazebo utility to test multi floor localization|
|[mf_localization_mapping](../mf_localization_mapping)|mapping function for multi floor localization.|
|[mf_localization_msgs](../mf_localization_msgs)|message difinitions for multi floor localization|
|[mf_localization_rviz](../mf_localization_rviz)|rviz plugins for multi floor localization control (floor up/down, restart localization)|
|[queue_msgs](../queue_msgs)|msgs for queue packages|
|[queue_people_py](../queue_people_py)|publish queue message to control robot in queue|
|[queue_utils_py](../queue_utils_py)|utilities for queue|


## Test

### Preparation

- run the script to download dependencies

```
./setup-dependency.sh
```

- run the script to build image and workspaces

```
./build-docker.sh
```


# License

[MIT License](LICENSE)


---
The following files/folder are under Apache-2.0 License

- cabot_description/urdf/sensors/_d435.gazebo.xacro
- cabot_description/urdf/sensors/_d435.urdf.xacro
- cabot_navigation/launch/cartographer_mapping.launch
