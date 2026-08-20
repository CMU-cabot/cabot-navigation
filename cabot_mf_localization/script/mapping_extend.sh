#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_NAME=$(basename "$0")

die() {
    echo "$SCRIPT_NAME: $*" >&2
    exit 1
}

bool_value() {
    case "$1" in
        true|false) printf '%s' "$1" ;;
        *) die "expected true or false, got '$1'" ;;
    esac
}

: "${EXTEND_BAG_FILENAME:?EXTEND_BAG_FILENAME is required}"
: "${EXTEND_LOAD_STATE_FILENAME:?EXTEND_LOAD_STATE_FILENAME is required}"
: "${EXTEND_OUTPUT_PREFIX:?EXTEND_OUTPUT_PREFIX is required}"

configuration_directory=${EXTEND_CONFIGURATION_DIRECTORY:-/home/developer/mapping_ws/src/mf_localization_mapping/configuration_files/cartographer}
configuration_basename=${EXTEND_CONFIGURATION_BASENAME:-}
mapping_launch_file=${EXTEND_MAPPING_LAUNCH_FILE:-/home/developer/mapping_ws/src/mf_localization_mapping/launch/demo_2d_VLP16.launch.py}
mf_localization_source_dir=${EXTEND_MF_LOCALIZATION_SOURCE_DIR:-/home/developer/mapping_ws/src/mf_localization}
global_initializer=${EXTEND_GLOBAL_INITIALIZER:-/home/developer/mapping_ws/src/cabot_mf_localization/script/mapping_global_initializer.py}
initial_pose_frame=${EXTEND_INITIAL_POSE_FRAME:-map}
use_initial_pose=$(bool_value "${EXTEND_USE_INITIAL_POSE:-false}")
load_frozen_state=$(bool_value "${EXTEND_LOAD_FROZEN_STATE:-true}")
launch_rviz=$(bool_value "${EXTEND_LAUNCH_RVIZ:-false}")
rate=${EXTEND_RATE:-1.0}
delay=${EXTEND_DELAY:-5}
grid_resolution=${EXTEND_GRID_RESOLUTION:-0.1}
shutdown_timeout=${EXTEND_SHUTDOWN_TIMEOUT:-3600}
global_init_play_seconds=${EXTEND_GLOBAL_INIT_PLAY_SECONDS:-30}
global_init_timeout=${EXTEND_GLOBAL_INIT_TIMEOUT:-3600}
global_init_pending_wait=${EXTEND_GLOBAL_INIT_PENDING_WAIT:-60}
points2=${EXTEND_POINTS2:-velodyne_points}
scan=${EXTEND_SCAN:-velodyne_scan}
imu=${EXTEND_IMU:-cabot/imu/data}
exclude_wifi_sample_pattern=${EXTEND_EXCLUDE_WIFI_SAMPLE_PATTERN-AIS-K25}
cabot_model=${CABOT_MODEL:-}

if [[ "$use_initial_pose" == true ]]; then
    : "${EXTEND_INITIAL_X:?EXTEND_INITIAL_X is required when EXTEND_USE_INITIAL_POSE=true}"
    : "${EXTEND_INITIAL_Y:?EXTEND_INITIAL_Y is required when EXTEND_USE_INITIAL_POSE=true}"
    : "${EXTEND_INITIAL_YAW:?EXTEND_INITIAL_YAW is required when EXTEND_USE_INITIAL_POSE=true}"
    : "${EXTEND_RELATIVE_TRAJECTORY_ID:?EXTEND_RELATIVE_TRAJECTORY_ID is required when EXTEND_USE_INITIAL_POSE=true}"
    [[ "$initial_pose_frame" == map || "$initial_pose_frame" == relative ]] \
        || die "EXTEND_INITIAL_POSE_FRAME must be 'map' or 'relative'"
    collect_metrics=false
    if [[ -z "$configuration_basename" ]]; then
        configuration_basename=cartographer_2d_mapping_extend.lua
    fi
else
    collect_metrics=true
    if [[ -z "$configuration_basename" ]]; then
        configuration_basename=cartographer_2d_mapping_extend_global.lua
    fi
fi
[[ -n "$cabot_model" ]] || die "CABOT_MODEL is required"

configuration_file=$configuration_basename
if [[ "$configuration_file" != /* ]]; then
    configuration_file="$configuration_directory/$configuration_file"
fi

[[ -f "$EXTEND_LOAD_STATE_FILENAME" ]] \
    || die "existing pbstream not found: $EXTEND_LOAD_STATE_FILENAME"
[[ -f "$EXTEND_BAG_FILENAME/metadata.yaml" ]] \
    || die "bag metadata not found: $EXTEND_BAG_FILENAME/metadata.yaml"
[[ -f "$configuration_file" ]] \
    || die "Cartographer configuration not found: $configuration_file"
[[ -f "$mapping_launch_file" ]] \
    || die "mapping launch not found: $mapping_launch_file"
if [[ "$use_initial_pose" == false ]]; then
    [[ -f "$global_initializer" ]] \
        || die "global initializer not found: $global_initializer"
fi

base_samples_filename=${EXTEND_BASE_SAMPLES_FILENAME:-}
if [[ -z "$base_samples_filename" ]]; then
    for candidate in \
        "${EXTEND_LOAD_STATE_FILENAME%.pbstream}.loc.samples.json" \
        "${EXTEND_LOAD_STATE_FILENAME%.pbstream}.base.loc.samples.json"; do
        if [[ -f "$candidate" ]]; then
            base_samples_filename=$candidate
            break
        fi
    done
fi
[[ -f "$base_samples_filename" ]] || die \
    "base samples not found; set EXTEND_BASE_SAMPLES_FILENAME"

python3 - \
    "$rate" \
    "$delay" \
    "$grid_resolution" \
    "$shutdown_timeout" \
    "$global_init_play_seconds" \
    "$global_init_timeout" \
    "$global_init_pending_wait" <<'PY'
import sys

(
    rate,
    delay,
    resolution,
    shutdown_timeout,
    global_init_play_seconds,
    global_init_timeout,
    global_init_pending_wait,
) = map(float, sys.argv[1:])
if not 0 < rate <= 1.0:
    raise SystemExit("EXTEND_RATE must be greater than 0 and no greater than 1.0")
if delay < 0:
    raise SystemExit("EXTEND_DELAY must be non-negative")
if resolution <= 0:
    raise SystemExit("EXTEND_GRID_RESOLUTION must be greater than 0")
if shutdown_timeout <= 0 or not shutdown_timeout.is_integer():
    raise SystemExit("EXTEND_SHUTDOWN_TIMEOUT must be a positive integer")
if global_init_play_seconds <= 0:
    raise SystemExit("EXTEND_GLOBAL_INIT_PLAY_SECONDS must be positive")
if global_init_timeout <= 0:
    raise SystemExit("EXTEND_GLOBAL_INIT_TIMEOUT must be positive")
if global_init_pending_wait <= 0:
    raise SystemExit("EXTEND_GLOBAL_INIT_PENDING_WAIT must be positive")
PY

output_prefix=$(realpath -m "$EXTEND_OUTPUT_PREFIX")
output_directory=$(dirname "$output_prefix")
output_filestem=$(basename "$output_prefix")
mkdir -p "$output_directory"

for suffix in \
    pbstream pgm png yaml info.txt \
    loc.samples.json loc.samples.json.fil.json; do
    [[ ! -e "$output_prefix.$suffix" ]] \
        || die "output already exists: $output_prefix.$suffix"
done

launch_pid=''
bag_pid=''
listener_pids=()
samples_tmpdir=$(mktemp -d "$output_directory/.${output_filestem}.samples.XXXXXX")
additional_samples_filename="$samples_tmpdir/additional.loc.samples.json"
tf2_beacons_log="$output_prefix.tf2_beacons.log"

stop_bag() {
    if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
        kill -INT "$bag_pid" 2>/dev/null || true
        wait "$bag_pid" 2>/dev/null || true
    fi
    bag_pid=''
}

stop_listeners() {
    local pid
    for pid in "${listener_pids[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    for pid in "${listener_pids[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    listener_pids=()
}

signal_launch_processes() {
    local signal=$1 pid
    while read -r pid; do
        [[ "$pid" =~ ^[0-9]+$ ]] || continue
        kill "-$signal" "$pid" 2>/dev/null || true
    done < <(ps -o pid= --ppid "$launch_pid" 2>/dev/null)
    kill "-$signal" "$launch_pid" 2>/dev/null || true
}

stop_launch() {
    local state
    if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
        # A command started as an asynchronous Bash job can inherit SIGINT as
        # ignored. Signal the processes launched by ros2 launch explicitly so
        # Cartographer runs final optimization and writes the PBStream.
        signal_launch_processes INT
        for _ in $(seq 1 "$shutdown_timeout"); do
            if ! kill -0 "$launch_pid" 2>/dev/null; then
                break
            fi
            state=$(ps -o stat= -p "$launch_pid" 2>/dev/null || true)
            [[ "$state" == Z* ]] && break
            sleep 1
        done
        if kill -0 "$launch_pid" 2>/dev/null; then
            echo "mapping launch did not stop after ${shutdown_timeout}s; terminating it" >&2
            signal_launch_processes TERM
        fi
        wait "$launch_pid" 2>/dev/null || true
    fi
    launch_pid=''
}

cleanup() {
    stop_bag
    stop_listeners
    stop_launch
    if [[ -d "$samples_tmpdir" ]]; then
        rm -r -- "$samples_tmpdir"
    fi
}
trap cleanup EXIT INT TERM

wait_for_services() {
    local service
    for _ in $(seq 1 120); do
        if ! kill -0 "$launch_pid" 2>/dev/null; then
            die "mapping launch exited before Cartographer services became available"
        fi
        for service in /start_trajectory /trajectory_query /finish_trajectory; do
            if ! ros2 service list 2>/dev/null | grep -Fxq "$service"; then
                sleep 1
                continue 2
            fi
        done
        return 0
    done
    die "timed out waiting for Cartographer services"
}

refresh_listener_trajectory() {
    local before after response
    before=$(grep -c 'Retrieved .* trajectory nodes from trajectory' \
        "$tf2_beacons_log" 2>/dev/null || true)
    before=${before:-0}
    if ! response=$(ros2 service call \
        /tf2_beacons_listener/call_trajectory_query \
        std_srvs/srv/Trigger '{}' 2>&1); then
        echo "warning: final trajectory query request failed: $response" >&2
        return
    fi
    for _ in $(seq 1 100); do
        after=$(grep -c 'Retrieved .* trajectory nodes from trajectory' \
            "$tf2_beacons_log" 2>/dev/null || true)
        after=${after:-0}
        if ((after > before)); then
            echo "final trajectory query completed before saving samples"
            return
        fi
        sleep 0.1
    done
    echo "warning: timed out waiting for the final trajectory query; " \
        "samples near the end of the bag may be omitted" >&2
}

compute_relative_pose() {
    python3 - \
        "$EXTEND_RELATIVE_TRAJECTORY_ID" \
        "$EXTEND_INITIAL_X" \
        "$EXTEND_INITIAL_Y" \
        "$EXTEND_INITIAL_YAW" \
        "$initial_pose_frame" <<'PY'
import math
import sys

import rclpy
from cartographer_ros_msgs.srv import TrajectoryQuery

trajectory_id = int(sys.argv[1])
x = float(sys.argv[2])
y = float(sys.argv[3])
yaw = float(sys.argv[4])
pose_frame = sys.argv[5]

if pose_frame == "relative":
    relative_x, relative_y, relative_yaw = x, y, yaw
else:
    rclpy.init()
    node = rclpy.create_node("mapping_extend_pose_query")
    client = node.create_client(TrajectoryQuery, "/trajectory_query")
    if not client.wait_for_service(timeout_sec=30.0):
        raise SystemExit("/trajectory_query is not available")
    request = TrajectoryQuery.Request()
    request.trajectory_id = trajectory_id
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
    response = future.result()
    if response is None or response.status.code != 0:
        raise SystemExit(f"trajectory query failed for trajectory {trajectory_id}")
    if not response.trajectory:
        raise SystemExit(f"trajectory {trajectory_id} has no poses")

    reference = response.trajectory[0].pose
    q = reference.orientation
    reference_yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    dx = x - reference.position.x
    dy = y - reference.position.y
    c = math.cos(reference_yaw)
    s = math.sin(reference_yaw)
    relative_x = c * dx + s * dy
    relative_y = -s * dx + c * dy
    relative_yaw = math.atan2(
        math.sin(yaw - reference_yaw),
        math.cos(yaw - reference_yaw),
    )
    print(
        "reference trajectory "
        f"{trajectory_id} first pose: "
        f"x={reference.position.x:.9f} y={reference.position.y:.9f} "
        f"yaw={reference_yaw:.9f}",
        file=sys.stderr,
    )
    node.destroy_node()
    rclpy.shutdown()

qz = math.sin(relative_yaw / 2.0)
qw = math.cos(relative_yaw / 2.0)
print(f"{relative_x:.12f} {relative_y:.12f} {qz:.12f} {qw:.12f}")
PY
}

finish_trajectory() {
    local trajectory_id=$1 response
    response=$(ros2 service call /finish_trajectory \
        cartographer_ros_msgs/srv/FinishTrajectory \
        "trajectory_id: $trajectory_id" 2>&1)
    printf '%s\n' "$response"
    grep -Eq 'code[=:][[:space:]]*0' <<<"$response" \
        || die "finish_trajectory failed for trajectory $trajectory_id"
}

echo "loading: $EXTEND_LOAD_STATE_FILENAME"
echo "bag: $EXTEND_BAG_FILENAME"
echo "Cartographer node and added trajectory configuration: $configuration_file"
echo "load frozen state: $load_frozen_state"
if [[ "$use_initial_pose" == true ]]; then
    echo "initial $initial_pose_frame pose: " \
        "x=$EXTEND_INITIAL_X y=$EXTEND_INITIAL_Y yaw=$EXTEND_INITIAL_YAW"
else
    echo "initial pose: disabled; waiting for a global inter-trajectory constraint"
fi

ros2 launch "$mapping_launch_file" \
    bag_filename:="$output_prefix" \
    load_state_filename:="$EXTEND_LOAD_STATE_FILENAME" \
    load_frozen_state:="$load_frozen_state" \
    configuration_directory:="$configuration_directory" \
    configuration_basename:="$configuration_basename" \
    collect_metrics:="$collect_metrics" \
    save_state:=true \
    save_samples:=false \
    save_trajectory:=false \
    save_pose:=false \
    launch_rviz:="$launch_rviz" \
    run_gnss_nodes:=false \
    use_sim_time:=true \
    points2:="$points2" \
    scan:="$scan" \
    imu:="$imu" \
    convert_points:=false \
    convert_imu:=false \
    convert_esp32:=false \
    cabot_model:="$cabot_model" \
    play_bag:=false \
    quit_when_rosbag_finish:=false \
    > >(tee "$output_prefix.log") 2>&1 &
launch_pid=$!
wait_for_services

if [[ "$use_initial_pose" == true ]]; then
    read -r relative_x relative_y initial_qz initial_qw < <(compute_relative_pose)
    echo "initial pose relative to trajectory $EXTEND_RELATIVE_TRAJECTORY_ID: " \
        "x=$relative_x y=$relative_y qz=$initial_qz qw=$initial_qw"
    request=$(cat <<EOF
configuration_directory: $configuration_directory
configuration_basename: $configuration_basename
use_initial_pose: true
initial_pose:
  position: {x: $relative_x, y: $relative_y, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: $initial_qz, w: $initial_qw}
relative_to_trajectory_id: $EXTEND_RELATIVE_TRAJECTORY_ID
EOF
    )
else
    request=$(cat <<EOF
configuration_directory: $configuration_directory
configuration_basename: $configuration_basename
use_initial_pose: false
EOF
    )
fi
response=$(ros2 service call /start_trajectory \
    cartographer_ros_msgs/srv/StartTrajectory "$request" 2>&1)
printf '%s\n' "$response"
grep -Eq 'code[=:][[:space:]]*0' <<<"$response" \
    || die "start_trajectory failed"
added_trajectory_id=$(sed -nE \
    's/.*trajectory_id[=:][[:space:]]*([0-9]+).*/\1/p' \
    <<<"$response" | tail -n 1)
[[ "$added_trajectory_id" =~ ^[0-9]+$ ]] \
    || die "could not determine the added trajectory ID"
echo "added trajectory ID: $added_trajectory_id"

python3 "$mf_localization_source_dir/script/tf2_beacons_listener.py" \
    --ros-args \
    -p use_sim_time:=true \
    -p "output:=$additional_samples_filename" \
    -p "topics:=['/esp32/wifi','/wireless/beacons','/wireless/wifi']" \
    -p save_empty_beacon_sample:=true \
    -p "output_trajectory:=$output_prefix.trajectory.csv" \
    -p trajectory_recorder_timer_period:=10.0 \
    -p "trajectory_id:=$added_trajectory_id" \
    -p interpolate_by_trajectory:=true \
    > >(tee "$tf2_beacons_log") 2>&1 &
listener_pids+=("$!")

python3 "$mf_localization_source_dir/script/tracked_pose_listener.py" \
    --ros-args \
    -p use_sim_time:=true \
    -p "output:=$output_prefix.tracked_pose.csv" \
    > >(tee "$output_prefix.tracked_pose.log") 2>&1 &
listener_pids+=("$!")

play_topics=(
    "/${points2#/}"
    "/${scan#/}"
    "/${imu#/}"
    /velodyne_packets
    /beacons
    /esp32/wifi
    /wireless/beacons
    /wireless/wifi
)

echo "playing additional bag at ${rate}x"
bag_args=(ros2 bag play --clock --rate "$rate")
if [[ "$delay" != 0 ]]; then
    bag_args+=(-d "$delay")
fi
bag_args+=(--topics "${play_topics[@]}" -- "$EXTEND_BAG_FILENAME")
"${bag_args[@]}" > >(tee "$output_prefix.bag.log") 2>&1 &
bag_pid=$!
if [[ "$use_initial_pose" == false ]]; then
    python3 "$global_initializer" \
        --play-seconds "$global_init_play_seconds" \
        --startup-delay "$delay" \
        --rate "$rate" \
        --timeout "$global_init_timeout" \
        --pending-constraint-wait "$global_init_pending_wait"
fi
if wait "$bag_pid"; then
    bag_status=0
else
    bag_status=$?
fi
bag_pid=''
((bag_status == 0)) || die "bag playback failed with status $bag_status"

refresh_listener_trajectory
stop_listeners

echo "finishing added trajectory $added_trajectory_id"
finish_trajectory "$added_trajectory_id"
echo "running final optimization and writing the combined state on shutdown"
stop_launch

[[ -s "$output_prefix.pbstream" ]] \
    || die "combined pbstream was not created: $output_prefix.pbstream"
if [[ ! -e "$additional_samples_filename" ]]; then
    printf '[]\n' > "$additional_samples_filename"
fi
merge_samples_args=(
    --output "$output_prefix.loc.samples.json"
    --filtered-output "$output_prefix.loc.samples.json.fil.json"
)
if [[ -n "$exclude_wifi_sample_pattern" ]]; then
    merge_samples_args+=(
        --exclude-wifi-sample-pattern "$exclude_wifi_sample_pattern"
    )
fi
merge_samples_args+=(
    "$base_samples_filename"
    "$additional_samples_filename"
)
python3 "$mf_localization_source_dir/script/merge_loc_samples.py" \
    "${merge_samples_args[@]}"

pushd "$output_directory" >/dev/null
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename "$output_filestem.pbstream" \
    -map_filestem "$output_filestem" \
    -resolution "$grid_resolution"
convert "$output_filestem.pgm" "$output_filestem.png"
ros2 run mf_localization_mapping extract_floormap_info_from_yaml.py \
    --input "$output_filestem.yaml" \
    --output "$output_filestem.info.txt"
identify -format 'width: %w\nheight: %h\n' "$output_filestem.pgm" \
    | tee -a "$output_filestem.info.txt"
popd >/dev/null

echo "combined pbstream: $output_prefix.pbstream"
echo "combined map:      $output_prefix.png"
echo "combined samples:  $output_prefix.loc.samples.json"
echo "filtered samples:  $output_prefix.loc.samples.json.fil.json"
