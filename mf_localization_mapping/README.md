# mf_localization_mapping package

Provide mapping function for multi-floor localization.

## configuration_files

configuration files for cartographer

## Extending a loaded map from a bag

`mapping_extend.sh` loads an existing PBStream with its trajectories frozen,
adds one trajectory from a bag at real-time speed, runs final optimization, and
writes the extended PBStream and map files. It also merges localization samples
from the loaded map and the added trajectory. Both the complete merged samples
(`.loc.samples.json`) and a filtered copy (`.loc.samples.json.fil.json`) are
written. By default, the filtered copy removes an entire sample when one of its
WiFi access-point IDs contains `AIS-K25`.

By default the added trajectory has no initial pose. Cartographer starts it in
an independent local frame and uses full-submap global constraints to connect
it to the loaded map. The script plays a short initialization window, pauses
the bag while global constraint work drains, and resumes only after the new
trajectory is connected. If the short segment finds a constraint before the
pose graph reaches its optimization interval, the script collects another
bounded segment and pauses again. This prevents an unbounded amount of later
bag data from accumulating as expensive global searches:

```bash
CABOT_MODEL=<model> \
EXTEND_BAG_FILENAME=/data/recording \
EXTEND_LOAD_STATE_FILENAME=/data/base.pbstream \
EXTEND_BASE_SAMPLES_FILENAME=/data/base.loc.samples.json \
EXTEND_OUTPUT_PREFIX=/data/output/extended \
EXTEND_RATE=1.0 \
ros2 run cabot_mf_localization mapping_extend.sh
```

The WiFi sample exclusion is a regular expression configured with
`EXTEND_EXCLUDE_WIFI_SAMPLE_PATTERN`. Set it to an empty string to keep every
sample in the filtered copy, or set a different expression for another robot
access-point naming convention.

The initialization window defaults to 30 seconds. It can be changed with
`EXTEND_GLOBAL_INIT_PLAY_SECONDS`; `EXTEND_GLOBAL_INIT_TIMEOUT` controls how
long the script waits for global matching (one hour by default). If a window
produces no match, the script resumes for another window and tries again. The
wait used to distinguish pending constraint work from an optimization that has
not yet been triggered defaults to 60 seconds and can be set with
`EXTEND_GLOBAL_INIT_PENDING_WAIT`.

Seedless extension uses `cartographer_2d_mapping_extend_global.lua`. Its global
matcher searches complete submaps while the trajectories are disconnected;
after the first connection it returns to the normal, narrower local search
window and does not reopen full-submap search during an ordinary recording.
This avoids late false matches between distant, visually similar areas. Seeded
extension uses `cartographer_2d_mapping_extend.lua`, which keeps the explicit
20 m and full-yaw local window for correcting an approximate pose.

Do not set `EXTEND_RATE` above `1.0`; the script rejects faster playback. The
Cartographer configuration is applied both when the node starts and when the
new trajectory starts because pose-graph options are only created at node
startup. After playback, the script waits for one final trajectory query before
stopping the sample listener so observations at the end of the bag are also
interpolated.

The extension configurations optimize every 90 trajectory nodes. Optimizing a
large loaded pose graph every few nodes blocks ingestion of high-rate IMU data
and can create a large work queue; the first node is still optimized
immediately by the Cartographer pose-graph implementation.

For a seeded search, add the following variables. A `map` pose is converted to
a pose relative to the selected loaded trajectory before mapping starts:

```bash
EXTEND_USE_INITIAL_POSE=true \
EXTEND_INITIAL_POSE_FRAME=map \
EXTEND_INITIAL_X=<x> \
EXTEND_INITIAL_Y=<y> \
EXTEND_INITIAL_YAW=<yaw-radians> \
EXTEND_RELATIVE_TRAJECTORY_ID=<trajectory-id>
```

The default configuration searches 20 m and the full yaw range while the
seeded trajectories are connected. If no seed is supplied, global matching is
used immediately; after a successful global constraint, subsequent matching
uses the local window.
