#!/bin/bash

# Copyright (c) 2024  Carnegie Mellon University
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


## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    blue "trap cabot_exploration.sh "

    kill -INT -1
    for pid in ${pids[@]}; do
	count=0
        while kill -0 $pid 2> /dev/null; do
	    if [[ $count -eq 10 ]]; then
		blue "escalate to SIGTERM $pid"
		com="kill -TERM $pid"
		eval $com
	    fi
	    if [[ $count -eq 20 ]]; then
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


## todo duplicate code
function err {
    >&2 red "[ERROR] "$@
}
function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}
function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

## private variables
pids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
command=''
commandpost='&'

### default variables
: ${CABOT_GAZEBO:=1}

use_sim_time=false
if [[ $CABOT_GAZEBO -eq 1 ]]; then
    use_sim_time=true
fi
points2_topic='/velodyne_points'
imu_topic='/cabot/imu/data'
bagoption=

### usage print function
function usage {
    echo "Usage"
    echo "    run this script after running cabot.sh in another terminal"
    exit
}

while getopts "h" arg; do
    case $arg in
    h)
        usage
        exit
        ;;
  esac
done
shift $((OPTIND-1))


### launch cartographer nonde for explore
com="$command ros2 launch cabot_exploration cabot_exploration.launch.yaml \
    use_sim_time:=$use_sim_time \
    configuration_basename:=cartographer_2d_explore.lua \
    points2_topic:=$points2_topic \
    imu_topic:=$imu_topic \
    $bagoption \
    $commandpost"
echo $com
eval $com
pids+=($!)

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    snore 1
done