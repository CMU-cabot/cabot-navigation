# cabot_navigation2 test

This folder includes both test files for `colcon test`, usual unittest for ROS2 and an original test script (run_test.py) to run integrated test on the simulater.

## Run the test cases on the simulator

```
# at the cabot-navigation dir
./launch.sh -s -t       # run tests module (tests.py) in your `CABOT_SITE` with GUI

# -s option launch the simulator and then
# -t option launch the integrated test by calling `script/run_test.sh` in the container and
# finally the run_test.sh will run run_test.py
```

### other options (check `./launch.sh -h`)
```
# run test (-t) options
  -S <site>   override CABOT_SITE
  -D          debug print
  -f <test>   run test CABOT_SITE.<module>.<test>
  -L          list test modules
  -l          list test functions
  -T <module> run test CABOT_SITE.<module>

# examples

./launch.sh -s -t -H                                                    # run test without gui
./laumch.sh -s -t -S cabot_site_test_room                               # run tests module in `cabot_site_test_room`
./launch.sh -s -t -S cabot_site_test_room -T tests-people               # run tests-people module in `cabot_site_test_room`
./launch.sh -s -t -S cabot_site_test_room -L                            # list test modules in `cabot_site_test_room`
./launch.sh -s -t -S cabot_site_test_room -T tests-people -l            # list test funcions in the module in `cabot_site_test_room`
./launch.sh -s -t -S cabot_site_test_room -T tests-people -f "test1.*"  # list test funcions matches with regex "test1.*"
```

### Test module file location

- <CABOT_SITE>/<CABOT_SITE>/tests.py
- see [cabot_sites_test/cabot_site_test_room/cabot_site_test_room/tests.py](https://github.com/CMU-cabot/cabot_sites_test/blob/main/cabot_site_test_room/cabot_site_test_room/tests.py) as an example

## Development tips for writing test cases
The following is a best practice to write test cases
- Launch a simulator first without running test cases
    ```
    ./docker/clean_ws.sh          # clean workspace - it can avoid building error with debug option
    ./build-docker.sh -w -d       # build workspace with debug option (colcon build with `--symlink-install` option)
    ./launch.sh -s                # launch simulator
    ```
- In a different terminal, run test cases
- You don't need to rebuild the workspace when you change test cases thanks to debug build (--symlink-install)
- If you add a test module, run `/launch.sh build -d` in the container termainal
    ```
    docker compose exec navigation bash                                     # run bash in the container
    ./script/run_test.sh [<options>] <module> <test function name regex>    # run test
    /launch.sh build -d                                                     # rebuild with debug option
    ```
- test options
    ```
    -d          debug print  # (not -D)
    -L          list test modules
    -l          list test functions
    -r          retry until the test succeeds
    -w          wait ready (this is used only when this script is launch by host ./launch.sh to wait for the simulater gets ready)
    ```

## run_test.py

- This script runs the specified test functions in the specified module

### Important note

- If you have gzclient (Gazebo GUI), actors can stop somehow (especially after deleting actors)
- Problem seems not happen when gzlient is not used
  - There might be a race condition to transport data between gaserver and gzclient

### Test script example

```
def config(tester):       # called first to setup configuration of the test
    tester.config['init_x'] = -5.0
    tester.config['init_y'] = 0.0
    tester.config['init_z'] = 0.0
    tester.config['init_a'] = 90.0
    tester.config['init_floor'] = 3


def checks(tester):       # calles second to setup global check_topic_error
    tester.check_topic_error(
        topic="/cabot/activity_log",
        topic_type="cabot_msgs/msg/Log",
        condition="msg.category=='cabot/interface' and msg.text=='vibration' and msg.memo=='unknown'"
    )


def wait_ready(tester):   # called if -w is option is specified to wait the simulator ready
    tester.wait_localization_started()  # much faster
    # tester.wait_ready()  # wait all the system is ready, but sometimes it takes too long


def test1_cancel_navigation_and_another_navigation(tester):
    tester.reset_position()
    tester.goto_node('EDITOR_node_1495220210524')
    tester.wait_for(10)
    tester.cancel_navigation()
    tester.wait_for(10)
    tester.goto_node('EDITOR_node_1495563142750')
    tester.wait_navigation_arrived()
```
#### Test function names

- Any function without "_" as the first letter will be executed in alphabetical order
- function with "_" as the first letter will be treated as private function


### Tester action functions and keyward args
- `call_service(**kwargs)`: **wait** until the service is responded
    - `service` : service name
    - `service_type` : service type
    - `request` : yaml expression of the request
- `check_topic(**kwargs)` : **not wait** success if a matched topic is received within a test
    - `topic` : topic name
    - `topic_type` : topic type
    - `condition` : pyton snipet to check if the `msg` matches the condition
      - if matches, the result will be `True`
- `check_topic_error(**kwargs)` : **not wait**, fail if any matched topic is received, otherwise success within a test
    - `topic` : topic name
    - `topic_type` : topic type
    - `condition` : pyton snipet to check if the `msg` matches the condition
      - if matches, the result will be `False`
    - **Returns** a function to cancel this test earlier
- `clean_door()` : **wait** until all doors are cleared
    - clean all spwaned doors
- `delete_actor(**kwargs)` : **wait** until an actor is deleted
    - `name` : actor name
- `delete_door(**kwargs)` : **wait** until a door is deleted
    - `name` : door name
- `init_manager()` : **wait** until PedestrianManager is initialized
    - init PedestrianManager (see https://github.com/CMU-cabot/cabot-navigation/tree/main/pedestrian_plugin)
- `pub_topic(**kwargs)` : publish a topic message
    - `topic` : topic name
    - `topic_type` : topic type
    - `message` : topic data in yaml text format
- `reset_position(**kwargs)`: **wait** until completed. Move the robot to the specified initial location, restart localization, and wait until localize_status becomes TRACKING
    - `x`, `y`, `z`, and `a` (yaw angle in degree)
    - if not specified, the `tester.config` value is used
- `setup_actors(**kwargs)` : **wait* until completed. spawn/reuse actors to the specified position and orientation
    - `actors`: [`ActorDict`]
    - `ActorDict`:
      - `name` : name of the actor
      - `module` : python module to be used to control the actor
      - `params` :
        - `init_x`, `inity`, `init_z`, and `init_a` (yaw angle in degree)
        - and custome parameters (should be int/float/str type)
- `spawn_door(**kwargs)` : **wait** until a single door (1cm x 2m x 2m) obstacle is spawned
    - `name` : obstacle name
    - `x`, `y`, `z`: location
    - `yaw` : orientaion
- `spawn_obstacle(**kwargs)` : **wait** until an obstacle is spawned
    - `name` : obstacle name
    - `x`, `y`, `z`: location
    - `yaw` : orientaion
    - `width` : x axis length
    - `height` : y axis length
    - `depth` : z axis length
- `wait_topi(**kwargs)` : waits until the condition matches or timeout
    - `topic` : topic name
    - `topic_type` : topic type
    - `condition` : pyton snipet to check if the `msg` matches the condition
- `wait(**kwargs)` : wait for seconds
    - `seconds` : wait time in seconds

### Pedestrian simulation configurations
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
- `button_down(button)` : use `pub_topic` to publish button event with `button` number
- `cancel_navigation()` : use `pub_toipc` to publish navigation cancel event
- `check_collision()` : use `check_topic_error` to check if Collision message comes
- `check_navigation_arrived()` : use `check_topic` to check if the robot arrive the destination
- `check_position(**kwargs)` : use `wait_topic` with `/cabot/pose_log` to wait the robot position is in an area
    - `x`, `y` : position
    - `tolerance` : x-y tolerance
    - `floor` : floor
- `check_turn_towards()` : use `check_topic` to check if the robot starts turning
- `floor_change(diff)` : call `floor_change` to change the floor of the robot
- `goto_node(node_id)` : use `pub_topic` to publish navigation destination event with `node_id`
- `wait_for(seconds)` : alias of `wait(seconds=<float>)`
- `wait_goal(goalName)` : use `wait_topic` to wait until an internal goal is completed
- `wait_navigation_completed()` : use `wait_topic` to wait until a navigation is completed
- `wait_navigation_arrived()` : use `wait_topic` to wait until a navigation is arrived. The robot can turn after arrival ant then completed event is emitted
- `wait_ready()` : use `wait_topic` to wait until the system is ready
- `wait_turn_towards()` : use `wait_topic` to wait until the robot starts turning
