#!/bin/bash

# Copyright (c) 2025  IBM Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

function blue {
    echo -en "\033[36m"
    echo "$@"
    echo -en "\033[0m"
}


function red {
    echo -en "\033[31m"
    echo "$@"
    echo -en "\033[0m"
}

function usage {
    echo "Usage: $0 [<options>] <bag_dir>"
    echo "-h          show this help"
    echo "-r <rate>   play bag rate"
    echo "-s <offset> play bag offset, bigger than or equal to 0"
    echo "-o          run localization with odometry topic"
    echo ""
    echo "Required environment variables:"
    echo "CABOT_MODEL      robot model used for localization"
    echo "CABOT_SITE       site package name used to resolve map_config_file"
    echo "Optional environment variables:"
    echo "CABOT_SITE_TAGS            map tags override"
    echo "CABOT_SITE_PKG_DIR         site package search root"
    echo "CABOT_GLOBAL_LOCALIZER_RUN run the global localizer node when set to 1"
    echo "CABOT_GLOBAL_LOCALIZER_USE use the global localizer result when set to 1"
}

function bool_arg {
    if [[ ${1:-0} -eq 1 ]]; then
        echo true
    else
        echo false
    fi
}

function find_site_config {
    local site_name=$1
    local search_root=$2
    if [[ -z $search_root || ! -d $search_root ]]; then
        return 1
    fi

    find "$search_root" -path "*/$site_name/config/config.sh" | head -n 1
}

# topic mapping for localization
points2_topic='/velodyne_points'
imu_topic='/cabot/imu/data'
beacons_topic='/wireless/beacons'
wifi_topic='/esp32/wifi'
odom_topic='/cabot/odometry/filtered'
pressure_topic='/cabot/pressure'
gnss_fix_topic='/ublox/fix'
gnss_fix_velocity_topic='/ublox/fix_velocity'
fix_filtered_topic='/ublox/fix_filtered'

# parameters
rate=1.0
start_time=0.0
with_odom=0
robot=${CABOT_MODEL:-}
site=${CABOT_SITE:-}
tags=${CABOT_SITE_TAGS:-}
: ${CABOT_PRESSURE_AVAILABLE:=0}
: ${CABOT_USE_GNSS:=0}
: ${CABOT_GLOBAL_LOCALIZER_RUN:=0}
: ${CABOT_GLOBAL_LOCALIZER_USE:=0}

run_global_localizer=$CABOT_GLOBAL_LOCALIZER_RUN
use_global_localizer=$CABOT_GLOBAL_LOCALIZER_USE

while getopts "hr:s:o" arg; do
    case $arg in
        h)
            usage
            exit
            ;;
        r)
            rate=$OPTARG
            ;;
        s)
            start_time=$OPTARG
            ;;
        o)
            with_odom=1
            ;;
    esac
done
shift $((OPTIND-1))

bag=$1

if [[ -z $robot || -z $site || -z $bag ]]; then
    usage
    exit 1
fi

if [[ ! -d $bag ]]; then
    red "Bag directory does not exist: $bag"
    exit 1
fi

# find map config file
config_file=

if [[ -n ${CABOT_SITE_PKG_DIR:-} ]]; then
    config_file=$(find_site_config "$site" "$CABOT_SITE_PKG_DIR")
fi

if [[ -z $config_file ]]; then
    config_file=$(find_site_config "$site" "/home/developer/loc_ws/src/cabot_sites")
fi

if [[ -z $config_file ]]; then
    sitedir=$(ros2 pkg prefix "$site" 2>/dev/null)/share/"$site"
    config_file=$sitedir/config/config.sh
fi

if [[ ! -f $config_file ]]; then
    red "Site config was not found for site: $site"
    exit 1
fi

sitedir=$(dirname "$(dirname "$config_file")")
blue "Using site config: $config_file"

source "$config_file"

if [[ -z $map ]]; then
    red "Map was not resolved from site config: $config_file"
    exit 1
fi

blue "Using map_config_file: $map"

# run command
cmd=(
    ros2 launch -n mf_localization_mapping
    demo_multi_2d_VLP16_rss_localization.launch.py
    use_sim_time:=true
    map_config_file:="$map"
    bag_filename:="$bag"
    start_time:="$start_time"
    rate:="$rate"
    cabot_model:="$robot"
    points2:="$points2_topic"
    imu:="$imu_topic"
    odom:="$odom_topic"
    beacon_topic:="$beacons_topic"
    wifi_topic:="$wifi_topic"
    pressure_topic:="$pressure_topic"
    pressure_available:="$(bool_arg "$CABOT_PRESSURE_AVAILABLE")"
    use_gnss:="$(bool_arg "$CABOT_USE_GNSS")"
    gnss_fix:="$gnss_fix_topic"
    gnss_fix_velocity:="$gnss_fix_velocity_topic"
    run_global_localizer:="$(bool_arg "$run_global_localizer")"
    use_global_localizer:="$(bool_arg "$use_global_localizer")"

)

# localization with odometry topic and TF
if [[ $with_odom -eq 1 ]]; then
    cmd+=(
        multi_floor_config_filename:=multi_floor_manager_with_odom.yaml
        republish_odometry_filtered:=true
    )
fi

# override site tags
if [[ -n $tags ]]; then
    cmd+=(tags:="$tags")
fi

printf '%q ' "${cmd[@]}"
echo
exec "${cmd[@]}"
