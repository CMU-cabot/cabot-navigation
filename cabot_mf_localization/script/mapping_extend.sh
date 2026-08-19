#!/usr/bin/env bash

set -Eeuo pipefail

: "${EXTEND_BAG_FILENAME:?EXTEND_BAG_FILENAME is required}"
: "${EXTEND_LOAD_STATE_FILENAME:?EXTEND_LOAD_STATE_FILENAME is required}"
: "${EXTEND_OUTPUT_PREFIX:?EXTEND_OUTPUT_PREFIX is required}"
: "${EXTEND_INITIAL_X:?EXTEND_INITIAL_X is required}"
: "${EXTEND_INITIAL_Y:?EXTEND_INITIAL_Y is required}"
: "${EXTEND_INITIAL_YAW:?EXTEND_INITIAL_YAW is required}"
: "${EXTEND_RELATIVE_TRAJECTORY_ID:?EXTEND_RELATIVE_TRAJECTORY_ID is required}"

configuration_directory=${EXTEND_CONFIGURATION_DIRECTORY:-/home/developer/mapping_ws/src/mf_localization_mapping/configuration_files/cartographer}
configuration_basename=${EXTEND_CONFIGURATION_BASENAME:-cartographer_2d_mapping.lua}
configuration_file="$configuration_directory/$configuration_basename"
output_directory=$(dirname "$EXTEND_OUTPUT_PREFIX")
output_filestem=$(basename "$EXTEND_OUTPUT_PREFIX")

if [[ ! -f "$EXTEND_LOAD_STATE_FILENAME" ]]; then
    echo "existing pbstream not found: $EXTEND_LOAD_STATE_FILENAME" >&2
    exit 1
fi
if [[ ! -f "$EXTEND_BAG_FILENAME/metadata.yaml" ]]; then
    echo "bag metadata not found: $EXTEND_BAG_FILENAME/metadata.yaml" >&2
    exit 1
fi
if [[ ! -f "$configuration_file" ]]; then
    echo "Cartographer configuration not found: $configuration_file" >&2
    echo "Set EXTEND_CONFIGURATION_DIRECTORY and/or EXTEND_CONFIGURATION_BASENAME to select an override." >&2
    exit 1
fi
echo "using Cartographer configuration: $configuration_file"

read -r initial_qz initial_qw < <(python3 - "$EXTEND_INITIAL_YAW" <<'PY'
import math
import sys

yaw = float(sys.argv[1])
print(math.sin(yaw / 2.0), math.cos(yaw / 2.0))
PY
)

launch_pid=''
listener_pids=()
stop_listeners() {
    for pid in "${listener_pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    for pid in "${listener_pids[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    listener_pids=()
}
cleanup() {
    if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
        kill -INT "$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
    stop_listeners
}
trap cleanup EXIT INT TERM

echo "loading state and starting mapping launch"
ros2 launch /home/developer/mapping_ws/src/mf_localization_mapping/launch/demo_2d_VLP16.launch.py \
    bag_filename:="$EXTEND_BAG_FILENAME" \
    load_state_filename:="$EXTEND_LOAD_STATE_FILENAME" \
    save_state:=true \
    save_samples:=false \
    save_trajectory:=false \
    save_pose:=false \
    launch_rviz:=false \
    run_gnss_nodes:=false \
    use_sim_time:=true \
    points2:=velodyne_points \
    imu:=cabot/imu/data \
    convert_points:=false \
    convert_imu:=false \
    cabot_model:="${CABOT_MODEL:?CABOT_MODEL is required}" \
    play_limited_topics:=true \
    delay:="${EXTEND_DELAY:-10}" \
    quit_when_rosbag_finish:=true \
    rate:=1.0 \
    start:=0 \
    > >(tee "${EXTEND_OUTPUT_PREFIX}.log") 2>&1 &
launch_pid=$!

service_ready=false
for _ in $(seq 1 60); do
    if ! kill -0 "$launch_pid" 2>/dev/null; then
        echo "mapping launch exited before start_trajectory was available" >&2
        exit 1
    fi
    if ros2 service list 2>/dev/null | grep -Fxq '/start_trajectory'; then
        service_ready=true
        break
    fi
    sleep 1
done
if [[ "$service_ready" != true ]]; then
    echo "timed out waiting for /start_trajectory" >&2
    exit 1
fi

request=$(cat <<EOF
configuration_directory: $configuration_directory
configuration_basename: $configuration_basename
use_initial_pose: true
initial_pose:
  position: {x: $EXTEND_INITIAL_X, y: $EXTEND_INITIAL_Y, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: $initial_qz, w: $initial_qw}
relative_to_trajectory_id: $EXTEND_RELATIVE_TRAJECTORY_ID
EOF
)
echo "starting the additional trajectory"
response=$(ros2 service call /start_trajectory cartographer_ros_msgs/srv/StartTrajectory "$request" 2>&1)
printf '%s\n' "$response"
if ! grep -Eq 'code[=:][[:space:]]*0' <<<"$response"; then
    echo "start_trajectory failed" >&2
    exit 1
fi

# The new trajectory ID is assigned by Cartographer.  It is 1 when the input
# pbstream only contains trajectory 0, but it can be 2 or greater when the
# input already contains additional trajectories.
added_trajectory_id=$(sed -nE 's/.*trajectory_id[=:][[:space:]]*([0-9]+).*/\1/p' <<<"$response" | tail -n 1)
if [[ ! "$added_trajectory_id" =~ ^[0-9]+$ ]]; then
    echo "could not determine the added trajectory ID from start_trajectory response" >&2
    exit 1
fi
echo "additional trajectory ID: $added_trajectory_id"

# Run these listeners from the source tree so that the added trajectory ID can
# be queried even when the workspace install tree predates this helper.
python3 /home/developer/mapping_ws/src/mf_localization/script/tf2_beacons_listener.py \
    --ros-args \
    -p use_sim_time:=true \
    -p "output:=$EXTEND_OUTPUT_PREFIX.loc.samples.json" \
    -p "topics:=['/esp32/wifi','/wireless/beacons','/wireless/wifi']" \
    -p save_empty_beacon_sample:=true \
    -p "output_trajectory:=$EXTEND_OUTPUT_PREFIX.trajectory.csv" \
    -p trajectory_recorder_timer_period:=10.0 \
    -p "trajectory_id:=$added_trajectory_id" \
    -p interpolate_by_trajectory:=true \
    > >(tee "${EXTEND_OUTPUT_PREFIX}.tf2_beacons.log") 2>&1 &
listener_pids+=("$!")

python3 /home/developer/mapping_ws/src/mf_localization/script/tracked_pose_listener.py \
    --ros-args \
    -p use_sim_time:=true \
    -p "output:=$EXTEND_OUTPUT_PREFIX.tracked_pose.csv" \
    > >(tee "${EXTEND_OUTPUT_PREFIX}.tracked_pose.log") 2>&1 &
listener_pids+=("$!")

if wait "$launch_pid"; then
    launch_status=0
else
    launch_status=$?
fi
launch_pid=''
if ((launch_status != 0)); then
    if [[ ! -s "$EXTEND_OUTPUT_PREFIX.pbstream" ]]; then
        echo "mapping launch failed with status $launch_status" >&2
        exit "$launch_status"
    fi
    echo "mapping launch exited with status $launch_status after writing the pbstream; continuing" >&2
fi

stop_listeners

if [[ ! -s "$EXTEND_OUTPUT_PREFIX.pbstream" ]]; then
    echo "combined pbstream was not created: $EXTEND_OUTPUT_PREFIX.pbstream" >&2
    exit 1
fi

pushd "$output_directory" >/dev/null
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename "$output_filestem.pbstream" \
    -map_filestem "$output_filestem" \
    -resolution 0.1
convert "$output_filestem.pgm" "$output_filestem.png"
ros2 run mf_localization_mapping extract_floormap_info_from_yaml.py \
    --input "$output_filestem.yaml" \
    --output "$output_filestem.info.txt"
identify -format 'width: %w\nheight: %h\n' "$output_filestem.pgm" \
    | tee -a "$output_filestem.info.txt"
popd >/dev/null

echo "combined pbstream: $EXTEND_OUTPUT_PREFIX.pbstream"
echo "combined map image: $EXTEND_OUTPUT_PREFIX.png"
echo "combined samples:   $EXTEND_OUTPUT_PREFIX.loc.samples.json"
