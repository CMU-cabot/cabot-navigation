#!/bin/bash

# Copyright (c) 2023, 2025  Carnegie Mellon University and Miraikan
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

#
# cabot_gazebo.sh
# This script runs gazebo server and related nodes, and spawn the robot
# 

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

pids=()
termpids=()
checks=()

## debug
debug=0
command_prefix=''
command_postfix='&'

# load utility functions
source $scriptdir/cabot_util.sh

trap signal INT TERM

function signal() {
    blue "trap cabot_ros2.sh "

    # ps -Af
    kill -INT -1
    for pid in ${termpids[@]}; do
	kill -TERM $pid
    done
    for pid in ${pids[@]}; do
	count=0
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
	    # ps -Af
            snore 1
	    count=$((count+1))
        done
    done
    
    exit
}

# initialize environment variables
# required variables
: ${CABOT_SIDE:=left}
: ${CABOT_SITE:=}
: ${CABOT_MODEL:=}
: ${CABOT_USE_GNSS:=0}
# variables with default value
: ${CABOT_INITX:=0}
: ${CABOT_INITY:=0}
: ${CABOT_INITZ:=0}
: ${CABOT_INITA:=0}  # in degree
export CABOT_INITAR=$(echo "$CABOT_INITA * 3.1415926535 / 180.0" | bc -l)

# check required environment variables
error_flag=0
if [[ -z $CABOT_SITE ]]; then
    err "CABOT_SITE sould be configured"
    error_flag=1
fi
if [[ -z $CABOT_MODEL ]]; then
    err "CABOT_MODEL should be configured"
    error_flag=1
fi
if [[ $error_flag -ne 0 ]]; then
    exit
fi

# initialize local variables
wait_sec=0
gdb=false

function usage {
    echo "Usage"
    echo ""
    echo "-h       show this help"
    echo "-w <sec> wait to launch in sec"
    echo "-g       run with gdb"
    exit
}

while getopts "hw:g" arg; do
    case $arg in
	h)
	    usage
	    exit
	    ;;
	w)
	    wait_sec=$OPTARG
	    ;;
	g)
	    gdb=true
	    ;;
    esac
done
shift $((OPTIND-1))

sleep $wait_sec

while [ ${PWD##*/} != "ros2_ws" ]; do
    cd ..
done
ros2_ws=`pwd`
cd $ros2_ws
source /usr/share/gazebo/setup.bash
source $ros2_ws/install/setup.bash

# TODO: Remove cabot_site_configuration from shell script
if [ ! -z $CABOT_SITE ]; then
    gazebo=1  # need this for config.sh
    sitedir=`ros2 pkg prefix $CABOT_SITE`/share/$CABOT_SITE
    echo $sitedir
    source $sitedir/config/config.sh
    if [ "$map" == "" ]; then
	echo "Please check config/config.sh in site package ($sitedir) to set map and world"
	exit
    fi
else
    if [ "$map" == "" ]; then
	echo "-T <site> or -m <map> should be specified"
	exit
    fi
fi

echo "CABOT_SITE                : $CABOT_SITE"
echo "Map                       : $map"

blue "launch gazebo"
com="$command_prefix ros2 launch cabot_gazebo cabot2_gazebo.launch.py \
        gdb:=$gdb \
        model:=$CABOT_MODEL \
        world_file:=$world \
        wireless_config_file:=$wireless_config \
        use_gnss:=$CABOT_USE_GNSS \
        $command_postfix"
echo $com
eval $com
checks+=($!)
termpids+=($!)

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    # blue "snore"
    for pid in ${checks[@]}; do
        kill -0 $pid 2> /dev/null
	if [[ $? -ne 0 ]]; then
	    red "process (pid=$pid) is not running, please check logs"
	    exit
	fi
    done
    snore 1
done
