# cabot_navigation2 test

## run_test.py

- This script runs test cases from the test file in your `CABOT_SITE`

## Run the test cases

```
# at the cabot-navigation dir
./launch.sh -s -t
```

### Test case file location

- <CABOT_SITE>/test/tests.yaml

### Test case spec

```
config:
  init_x: <robot initial location>
  init_y: <robot initial location>

# specify check condition which should not happens all the time
checks: 
- name: <name of check>
  action:
    type: check_topic_error
    topic: <topic>
    topic_type: <topic_msgs/msg/MsgType>
    condition: "python snipet to check the msg content (variable msg is given)"

# test actions that will be executed
tests:
- comment: <comment message>
- name: <name of the action>
  action:
    type: <test_action>
    timeout: <timeout seconds> (default=60)
    <other parameters>
```

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