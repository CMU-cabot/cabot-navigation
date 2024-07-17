#!/bin/bash

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
pid=
image_pid=

# load utility functions
source $scriptdir/cabot_util.sh

trap signal INT TERM

function signal() {
    echo "trap record_bag.sh"
    kill -2 $pid
    if [[ $separate_log -eq 1 ]]; then
        kill -2 $image_pid
    fi

    while kill -0 $pid 2> /dev/null || ([[ $separate_log -eq 1 ]] && kill -0 $image_pid 2> /dev/null); do
        if [[ $count -eq 15 ]]; then
            blue "escalate to SIGTERM $pid"
            com="kill -TERM $pid"
            eval $com
            if [[ $separate_log -eq 1 ]]; then
                com="kill -TERM $image_pid"
                eval $com
            fi
        fi
        if [[ $count -eq 30 ]]; then
            blue "escalate to SIGKILL $pid"
            com="kill -KILL $pid"
            eval $com
            if [[ $separate_log -eq 1 ]]; then
                com="kill -KILL $image_pid"
                eval $com
            fi
        fi
        echo "waiting $0 $pid"
        snore 1
        count=$((count+1))
    done

    exit
}

echo "recording to $ROS_LOG_DIR"
source $scriptdir/../install/setup.bash

use_sim_time=
while getopts "s" arg; do
    case $arg in
	s)
	    use_sim_time="--use-sim-time"
	    ;;
    esac
done
shift $((OPTIND-1))

backend=${CABOT_ROSBAG_BACKEND:=sqlite3}
compression=${CABOT_ROSBAG_COMPRESSION:=message}
record_cam=${CABOT_ROSBAG_RECORD_CAMERA:=0}
separate_log=${CABOT_ROSBAG_SEPARATE_LOG:=0}


exclude_topics_file="rosbag2-exclude-topics.txt"
exclude_camera_topics="/.*/image_raw.*"
#hidden_topics="--include-hidden-topics"
hidden_topics="" # workaround - if this is specified with '-s mcap', only a few topics can be recorded
qos_option="--qos-profile-overrides-path $scriptdir/rosbag2-qos-profile-overrides.yaml"
if [[ $compression == "none" ]]; then
    compression=""
else
    compression="--compression-mode $compression --compression-format zstd"
fi

interval=1000

if [[ $record_cam -eq 1 ]]; then
    if [[ $separate_log -eq 1 ]]; then
        image_com="ros2 bag record ${use_sim_time} -s ${backend} ${compression} -p ${interval} -e '/.*camera_info|/.*/color/image_raw' -x '/.*image_raw$' ${hidden_topics} ${qos_option} -o $ROS_LOG_DIR/image_topics 2>&1 &"
        echo $image_com
        eval $image_com
        image_pid=$!
    else
        exclude_camera_topics='/.*/image_raw$'  # record compressed images
    fi
fi
lines=$(cat $scriptdir/${exclude_topics_file})
exclude_topics=$(echo -n "${lines}" | tr '\n' '|')
exclude_topics="${exclude_topics}|${exclude_camera_topics}"

if [[ -z $ROS_LOG_DIR ]]; then
    # for debug only
    com="ros2 bag record ${use_sim_time} -s ${backend} ${compression} -p ${interval} -a -x \"${exclude_topics}\" ${hidden_topics} ${qos_option} 2>&1 &"
else
    com="ros2 bag record ${use_sim_time} -s ${backend} ${compression} -p ${interval} -a -x \"${exclude_topics}\" ${hidden_topics} ${qos_option} -o $ROS_LOG_DIR/ros2_topics 2>&1 &"
fi
echo $com
eval $com
pid=$!

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    # blue "snore"
    snore 1
done
