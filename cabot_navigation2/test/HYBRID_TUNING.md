# Hybrid controller runtime tuning

`/controller_server` accepts live changes to the numeric `HybridRLFollowPath.*`
parameters. A complete committed snapshot is used for the next control cycle;
controller switching, lifecycle reconfiguration, and navigation cancellation are
not required. Topics and the plugin name require reconfiguration.

Examples after starting CaBot with the updated binaries:

```sh
ros2 param set /controller_server HybridRLFollowPath.max_linear_velocity 0.5
ros2 param set /controller_server HybridRLFollowPath.people_cost_wt 1.5
ros2 param set /controller_server HybridRLFollowPath.heading_cost_wt 0.3
```

All numeric values use the ROS double type, including `linear_sample_size` and
`angular_sample_size` (for example, `10.0`). Sample counts must be whole numbers
from 1 to 200. Values must be finite and nonnegative. Additional constraints:

- `0 < sampling_rate <= prediction_horizon` (both measured in seconds).
- `0 < lookahead_distance <= max_lookahead` (meters).
- `discount_factor` is in `[0, 1]`; `obstacle_costval` is in `[1, 255]`.
- A control cycle may predict at most 200,000 poses, including sampling both
  endpoints and the explicitly included zero angular velocity.

Use `/controller_server/set_parameters_atomically` when changing related values
such as the prediction horizon and time step together. Rejected transactions do
not affect the controller, including rejection by another node callback.

`heading_cost_wt` (default `0.3`) penalizes terminal heading error toward the
route's local goal, using the shortest angular distance. `angular_cost_wt`
(default `0.05`) penalizes unnecessary rotation. Tied candidates prefer smaller
angular velocity magnitude, then smaller translation. A goal to the side
or behind can still produce a necessary route-following turn. Nearby goals use the same
obstacle evaluation. Within `focus_goal_dist`, translation is capped to avoid
passing the local goal within the prediction horizon.

When MPC selects no translation toward a forward-facing route, a bounded
avoidance check compares exit rays on both sides. It checks the swept footprint,
costmap, and predicted people, then favors clearance, smaller route deviation,
the RL subgoal direction, and the previously selected side. An exit must remain
clear throughout the probe; when neither exit is clear the robot waits. The
command is a slow rotation in place, still subject to downstream speed limits.
The angular bound is measured from the route direction, so repeated attempts
cannot accumulate into a turn toward the back of the route. This does not
restrict a U-turn required by a new destination behind the robot.

These additional doubles are also live parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `avoidance_max_angle` | `0.7` | Maximum route deviation in radians (about 40 degrees); `0.0` disables avoidance turns, maximum pi/3. |
| `avoidance_angular_velocity` | `0.25` | Rotation cap in rad/s, additionally capped by `max_angular_velocity`; range 0 to 0.5. |
| `avoidance_probe_distance` | `1.5` | Exit ray length in meters, range 0.3 to 3.0. |
| `avoidance_min_clearance` | `0.65` | Minimum center distance to predicted people in meters, minimum 0.3. |

This is a short-range escape heuristic, not a complete detour planner. It does
not reduce the lidar, touch, or user-speed limits. A blocked exit or a person
within the clearance radius still causes a stop.

Live changes are not saved back to YAML. Persist intended defaults in
`params/nav2_params_hybrid.yaml`. The launch file currently overrides the initial
`max_linear_velocity` using `CABOT_INIT_SPEED`, then `CABOT_MAX_SPEED`, then `1.0`;
live parameter changes take effect after that startup override.

## Summons speed

Summons uses touch-to-stop and keeps `/cabot/user_speed_enabled` enabled. The
translation ceiling is the minimum of the controller output, `user_speed`, the
summons ceiling (default `1.0 m/s`), and the other enabled speed limits. Touch
stops both translation and rotation. Missing touch messages still trigger the
existing downstream timeout stop.

The summons ceiling can also be changed live:

```sh
ros2 param set /cabot/touch_speed_control_node touch_speed_max_speed_inactive 1.0
```

Its persistent setting is `touch_speed_max_speed_inactive` under
`cabot/config/cabot-control.yaml`. The old `touch_speed_max_inactive` spelling
was not read by the C++ touch node.

## Regression checks

Run tests with `ROS_DOMAIN_ID=93` and an NVMe-backed `TMPDIR`, separate from the
robot's live ROS domain. The C++ `test_hybrid_controller` target tests blocked
translation, valid turns, the ±pi boundary, near-goal obstacle checking, dynamic
parameter updates, rejected transactions, and fresh RL input for new routes.

`cabot_ui/test/test_summons_speed.py` checks the UI service requests and refuses
to start summons if its touch mode or user-speed limiter cannot be enabled.
`cabot/test/test_summons_limits.py` runs the actual touch and speed-limit node
executables with synthetic inputs, checking the minimum speed ceiling and
touch/timeout stops. Set `CABOT_TEST_BIN_DIR` and put the new `libsafety_nodes.so`
directory first in `LD_LIBRARY_PATH` to test a build before installation.
