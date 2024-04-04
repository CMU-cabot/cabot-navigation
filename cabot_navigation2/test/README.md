# cabot_navigation2 test

## run_test.py

- This script runs test cases from the test file in your `CABOT_SITE`

### Important note

- If you have gzclient (Gazebo GUI), actors can stop somehow (especially after deleting actors)
- Problem seems not happen when gzlient is not used
  - There might be a race condition to transport data between gaserver and gzclient

## Run the test cases

```
# at the cabot-navigation dir
./launch.sh -s -t
```
### Test script file location

- <CABOT_SITE>/test/tests.py
- see cabot_site_test/test/tests.py as an example

### Test script example

```
func config(tester: Tester)
func checks(tester: Tester)
func wait_ready(tester: Tester)
func test_name(tester: Tester)
```

#### Test function names

- Any function without "_" as the first letter will be executed in alphabetical order
- function with "_" as the first letter will be treated as private function

### Tester action functions and keyward args
- `init_manager`:
    - init PedestrianManager
- `check_topic_error`
    - `topic` : topic name
    - `topic_type` : topic type
    - `condition` : pyton snipet to check if the `msg` matches the condition
      - if matches, the result will be `False`
- `check_topic`
    - `topic` : topic name
    - `topic_type` : topic type
    - `condition` : pyton snipet to check if the `msg` matches the condition
      - if matches, the result will be `True`
- `wait_ready`
    - wait until the system is ready. It is equivalent to the following `wait_topic` action
        ```
        tester.wait_topic(
          topic="/cabot/activity_log"
          topic_type="cabot_msgs/msg/Log"
          condition="msg.category=='cabot/interface' and msg.text=='status' and msg.memo=='ready'"
        )
        ```
- `reset_position`: Move the robot to the specified initial location, restart localization, and wait until localize_status becomes TRACKING
    - `x`, `y`, `z`, and `a` (yaw angle in degree)
    - if not specified, the `tester.config` value is used
- `setup_actor` : spawn/reuse actors to the specified position and orientation
    - `actors`: [`ActorDict`]
    - `ActorDict`:
      - `name` : name of the actor
      - `module` : python module to be used to control the actor
      - `params` :
        - `init_x`, `inity`, `init_z`, and `init_a` (yaw angle in degree)
        - and custome parameters (should be int/float/str type)
- `pub_topic` : publish a topic message
    - `topic` : topic name
    - `topic_type` : topic type
    - `message` : topic data in yaml text format
- `wait_topic` : waits until the condition matches or timeout
    - `topic` : topic name
    - `topic_type` : topic type
    - `condition` : pyton snipet to check if the `msg` matches the condition
- `wait` : wait for seconds
    - `seconds` : wait time in seconds
- `set_evaluation_parameters(metrics=[], robot_radius=None)` : set parameters used for computing metrics
    - `metrics : Optional[list], default=[]` : list of metric functions to be computed. The callable functions are defined in [evaluation_metrics.py](evaluation_metrics.py).
        - Example
            ```
            metrics=[
                "total_time",
                "robot_path_length",
                "time_not_moving",
                "avg_robot_linear_speed",
                "cumulative_heading_changes"
            ]
            ```
        - Note: The implementation of the metric functions is not stable as it is under development.
    - `robot_radius : Optional[float], default=None` : robot radius used to detect collisions in metric computation. If not defined, the default value (0.45) defined in the pedestrian plugin will be used.
- `start_evaluation` : start computing the metrics. This method should be called when ready to start the navigation.
- `stop_evaluation` : stop computing the metrics. It is usually not necessary to call this method because it is automatically called when the test ends.

### Short hand for test

- `check_collision`
- `goto_node`
- `cancel_navigation`
- `button_down`
- `wait_navigation_completed`
- `wait_navigation_arrived`
- `wait_turn_towards`
- `check_navigation_arrived`
- `check_turn_towards`
- `wait_for`

### Test action types and params
- `wait_ready`
    - wait until the system is ready. It is equivalent to the following `wait_topic` action
        ```
        action:
          type: wait_topic
          topic: /cabot/activity_log
          topic_type: cabot_msgs/msg/Log
          condition: "msg.category=='cabot/interface' and msg.text=='status' and msg.memo=='ready'"
        ```
- `reset_position`: Move the robot to the specified initial location, restart localization, and wait until localize_status becomes TRACKING
    - `x`, `y`, `z`, and `a` (yaw angle in degree)
    - if not specified, the `config` value is used
- `spawn_actor` : spawn an actor to the specified position and orientation
    - `name` : name of the actor
    - `x`, `y`, `z`, and `a` (yaw angle in degree)
    - `module` : python module to be used to control the actor
    - `params` : Dict of parameters
- `delete_actor` : delete the specified actor
    - `name` : name of the actor
- `pub_topic` : publish a topic message
    - `topic` : topic name
    - `topic_type` : topic type
    - `message` : topic data in yaml text format
- `wait_topic` : waits until the condition matches or timeout
    - `topic` : topic name
    - `topic_type` : topic type
    - `condition` : pyton snipet to check if the `msg` matches the condition