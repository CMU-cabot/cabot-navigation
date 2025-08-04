#!/bin/bash

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
pids=()

# load utility functions
source $scriptdir/cabot_util.sh

trap signal INT TERM

function signal() {
    echo "trap play_bag.sh"
    for pid in ${pids[@]}; do
	kill -2 $pid
	while kill -0 $pid 2> /dev/null; do
	    if [[ $count -eq 15 ]]; then
		blue "escalate to SIGTERM $pid"
		com="kill -TERM $pid"
		eval $com
	    fi
	    if [[ $count -eq 30 ]]; then
		blue "escalate to SIGKILL $pid"
		com="kill -KILL $pid"
		eval $com
	    fi
            echo "waiting $0 $pid"
            snore 1
	    count=$((count+1))
	done
    done
    exit
}

# environment variables
: ${CABOT_SHOW_ROS2_LOCAL_RVIZ:=0}
: ${CABOT_ROS2_RVIZ_CONFIG}
: ${CABOT_ROS2_LOCAL_RVIZ_CONFIG}

source $scriptdir/../install/setup.bash

rate=1.0
start=0.01
while getopts "r:s:m:" arg; do
    case $arg in
        r)
            rate=$OPTARG
            ;;
	s)
	    start=$OPTARG
	    ;;
    esac
done
shift $((OPTIND-1))

bag=$1
if [[ -z $bag ]]; then
    echo "Usage: $0 <bag_file>"
    exit 1
fi
echo "play $bag"


if (( $(echo "$start > 0.01" | bc -l) )); then
    ros2 run cabot_debug print_topics.py -f $bag -i > /dev/null 2>&1
    if [[ $? -ne 0 ]]; then
        red "could not read the bag file, please check if the bag path is correct and the bag has correct metadata.yaml"
        exit
    fi
    map=$(ros2 run cabot_debug print_topics.py -f $bag -d $start -r -t /current_map_filename 2> /dev/null | tail -1)
    echo "last current_map_filename = $map"
    temp_str="${map#package://}"
    package="${temp_str%%/*}"
    echo "package = $package"
    prefix=$(ros2 pkg prefix $package)
    path="${temp_str#*/}"
    map_path="map:=$prefix/share/$package/$path"

    tf_frame="frame:=$(ros2 run cabot_debug print_topics.py -f $bag -d $start -r -t /current_frame 2> /dev/null | tail -1)"
    
    temp_file="$(mktemp)"
    ros2 run cabot_debug print_topics.py -f $bag -1 -r -t /robot_description > $temp_file
    temp_file="robot:=$temp_file"
fi

# republish uncompressed images to visualize in rviz2
readarray -t compressed_image_topics < <(ros2 bag info $bag | grep -oP '(?<=Topic: ).*(?=/compressed )')
for compressed_image_topic in "${compressed_image_topics[@]}"; do
    com="ros2 run image_transport republish compressed --ros-args -r in/compressed:=$compressed_image_topic/compressed -r out:=$compressed_image_topic &"
    echo $com
    eval $com
    pids+=($!)
done

rviz_option=
if [[ -n $CABOT_ROS2_RVIZ_CONFIG ]]; then
    rviz_option+=" rviz_config_file:=$CABOT_ROS2_RVIZ_CONFIG"
fi
if [[ -n $CABOT_ROS2_LOCAL_RVIZ_CONFIG ]]; then
    rviz_option+=" rviz_config_file2:=$CABOT_ROS2_LOCAL_RVIZ_CONFIG"
fi

com="ros2 launch cabot_debug play_bag.launch.py \
   bagfile:=$bag \
   start:=$start \
   rate:=$rate \
   $map_path \
   $temp_file \
   $tf_frame \
   show_local_rviz:=$CABOT_SHOW_ROS2_LOCAL_RVIZ \
   $rviz_option &"

echo $com
eval $com
pids+=($!)

if (( $(echo "$start > 0.01" | bc -l) )); then
    ros2 run cabot_debug print_topics.py -f $bag -d $start -t /global_costmap/costmap -P &
    pids+=($!)
fi

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    # blue "snore"
    snore 1
done
