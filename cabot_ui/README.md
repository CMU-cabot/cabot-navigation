# cabot_ui

- `cabot_ui_manager` is the node to manage the user interface and the navigation of the robot

## Events for navigation
- **topic**: `/cabot/event` (std_msgs/msg/String)
- Since these events are added on an ad-hoc basis and are not organized, a dedicated message would be better instead of a String message

### Navigation event

- Event string can be parsed with [cabot_ui/event.py](https://github.com/CMU-cabot/cabot-navigation/blob/main/cabot_ui/cabot_ui/event.py)

#### Publisher/Subscribers
- [cabot-ble-server](https://github.com/cmu-cabot/cabot-ble-server) (cabot-ios-app) `PUB`/`SUB`
- [cabot_speed_control_node.cpp](https://github.com/CMU-cabot/cabot-navigation/blob/main/cabot/src/safety/people_speed_control_node.cpp) `PUB`
- [cabot-ui-manager.py](https://github.com/CMU-cabot/cabot-navigation/blob/main/cabot_ui/scripts/cabot_ui_manager.py) `PUB`/`SUB`
- [BT Navigator - behavior trees](https://github.com/CMU-cabot/cabot-navigation/tree/main/cabot_bt/behavior_trees) `PUB`
- [stop_reasons_node.cpp](https://github.com/CMU-cabot/cabot-navigation/blob/main/cabot_ui/scripts/stop_reasons_node.py) `PUB`
- [run_test.py](https://github.com/CMU-cabot/cabot-navigation/blob/main/cabot_navigation2/test/run_test.py) `PUB`/`SUB` (for integration test)

#### Control event
- `navigation_next` (**OUT**): Requesting the destination set in the app
- `navigation_subtour` (**OUT**): Requesting the app to proceed subtour
- `navigation_destination;<NodeID>` (**IN**): Set the robot's destination to `NodeID`
- `navigation_summons;<NodeID>` (**IN**): Set the robot's destination to `NodeID` with summons mode
- `navigation_cancel` (**IN**): Cancel the navigation

#### Notification event
- `navigation_arrived` (**OUT**): Notify that the robot havs arrived at the destination
- `navigation_sound;<SoundID>` (**OUT**): Notify the sound output in the app

#### Language event
- `navigation_getlanguage` (**OUT**): Requesting the language set in the app
- `navigation_language;<Lang>` (**IN**): Set the robot's language to <Lang>
  - `en` or `ja`

#### Internal control event (mapped with button events but can be trigger by external input)
- `navigation_speedup` (**IN**): Increase robot speed by 0.05m/s (`button_down_1`)
- `navigation_speeddown` (**IN**): Decrease robot speed by 0.05m/s (`button_down_2`)
- `navigation_pause` (**IN**): Pause the current navigation (`button_down_3`)
- `navigation_resume` (**IN**): Resume the current navigation  (`button_down_4`)
- `navigation_idle` (**IN**): Stop loop control of the motor (`button_holddown_3`)
- `navigation_decision` (**IN**): Send `navigation_subtour` (`button_down_5`)

#### Internal notification event
- `navigation;event;<InternalNavigationEvent>` (**IN**): Notify the internal navigation event
  - `navigation_start`: Notify when the robot starts follows the global plan
  - `waiting_for_elevator`: Notify when the robot starts to wait the elevator
  - `elevator_door_may_be_ready`: Notify when the robot can make a path going into the elevator cabin
  - `people_speed_stopped`: Notify when people_speed_control_node limit the speed to zero
  - `people_speed_following`: Notify when people_speed_control_node limit the speed under 75% of max speed

### Click/Button event

- Event string can be parsed with [cabot_common/event.py](https://github.com/CMU-cabot/cabot-common/blob/main/cabot_common/cabot_common/event.py)

#### Publisher
- [cabot_hanel_v2_node.cpp](https://github.com/CMU-cabot/cabot-drivers/blob/main/cabot_base/src/cabot/cabot_handle_v2_node.cpp) `PUB`
- [cabot_keyboard.py](https://github.com/CMU-cabot/cabot-navigation/blob/main/cabot_ui/scripts/cabot_keyboard.py) `PUB` (for debug - xterm)

#### Click event
- `click_<Button>_<Count>`: click `<Button>` `<Count>` times within the click interval
- Example events
    ```
    button_down_3
    button_up_3
    click_3_1
    button_down_3
    button_up_3
    button_down_3
    button_up_3
    click_3_2
    ```

#### Button event

- `<Button>`
  - `1`: forward button
  - `2`: backward button
  - `3`: left button
  - `4`: right button
  - `5`: center button (optional)
- `button_down_<Button>`: button down
- `button_up_<Button>`: button up
- `button_hold_<Button>`: button hold


#### Handle Button Mapping
- specify button mapping in .env
  ```
  CABOT_HANDLE_BUTTON_MAPPING=2
  ```
##### 1 (Old: ~2025/02)
example .env file
```
CABOT_IMAGE_DESCRIPTION_ENABLED=true
CABOT_IMAGE_DESCRIPTION_SERVER=http://localhost:8000
CABOT_HANDLE_BUTTON_MAPPING=1
CABOT_IMAGE_DESCRIPTION_MODE=surround,stop-reason
```
Key Mapping
- Right Click: Start navigation
- Left Click: Pause navigation
- Down Click: decrease speed
- Up Click: increase speed
- Left Hold (3 seconds): Stop motor control
- Down Hold: Describe the surroundings or stop reason
  ```
  #CABOT_IMAGE_DESCRIPTION_MODE=surround  # describe surroundings（holding 1 second: short length, 2 seconds: medium, 3 seconds: long）
  #CABOT_IMAGE_DESCRIPTION_MODE=stop-reason  # only stop reason (holding 1 second)
  CABOT_IMAGE_DESCRIPTION_MODE=surround,stop-reason  # holding 1~2 seconds: stop reason, 3 seconds: surrounding (medium length)
  ```


##### 2 (Default)
example .env file
```
CABOT_IMAGE_DESCRIPTION_ENABLED=true
CABOT_IMAGE_DESCRIPTION_SERVER=http://localhost:8000
CABOT_HANDLE_BUTTON_MAPPING=2
CABOT_IMAGE_DESCRIPTION_MODE=surround,stop-reason
```
Key Mapping
- Left Hold (1 second): Pause navigation
- Left Hold (3 seconds): Stop motor control
- Down Hold: Gradually decrease speed (speed drops every second)
- Up Hold: Gradually increase speed (speed rises every second)
- Left Click: Pause/Resume speech output
- Right Click: Start/Resume navigation
  - *while navigating: Describe stop reason
- Down Click: Start conversation interface
- Up Click: Describe the surroundings
  - Single click: Short analysis
  - Double click: Medium-length analysis
  - Triple click: Detailed analysis
