#!/bin/bash

# Copyright (c) 2020, 2022  Carnegie Mellon University
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

set -m

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
    blue "trap cabot_gui.sh "

    # ps -Af
    for pid in ${termpids[@]}; do
        com="kill -TERM $pid"
        red $com
        eval $com
    done
    for pid in ${pids[@]}; do
        com="kill -SIGINT $pid"
        red $com
        eval $com
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

# TODO
: ${CABOT_GAZEBO:=0}
: ${CABOT_USE_SIM_TIME:=0}
: ${CABOT_USE_HANDLE_SIMULATOR:=0}
: ${CABOT_SHOW_GAZEBO_CLIENT:=0}
: ${CABOT_SHOW_ROS2_RVIZ:=0}
: ${CABOT_SHOW_ROS2_LOCAL_RVIZ:=0}
: ${CABOT_SHOW_ROBOT_MONITOR:=1}
: ${CABOT_GUI_TELEOP_TWIST_TOPIC:=cmd_vel}  # variable to remap cmd_vel topic for simulator

wait_sec=0

function usage {
    echo "Usage"
    echo ""
    echo "-h       show this help"
    echo "-w <sec> wait to launch in sec"
    exit
}

while getopts "hw:" arg; do
    case $arg in
        h)
            usage
            exit
            ;;
        w)
            wait_sec=$OPTARG
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

echo "CABOT_GAZEBO              : $CABOT_GAZEBO"
echo "CABOT_USE_SIM_TIME        : $CABOT_USE_SIM_TIME"
echo "CABOT_USE_HANDLE_SIMULATOR: $CABOT_USE_HANDLE_SIMULATOR"
echo "CABOT_SHOW_ROS2_RVIZ      : $CABOT_SHOW_ROS2_RVIZ"
echo "CABOT_SHOW_ROS2_LOCAL_RVIZ: $CABOT_SHOW_ROS2_LOCAL_RVIZ"
echo "CABOT_SHOW_GAZEBO_CLIENT  : $CABOT_SHOW_GAZEBO_CLIENT"
echo "CABOT_SHOW_ROBOT_MONITOR  : $CABOT_SHOW_ROBOT_MONITOR"
echo "CABOT_GUI_TELEOP_TWIST_TOPIC  : $CABOT_GUI_TELEOP_TWIST_TOPIC"
echo "CABOT_ROS2_RVIZ_CONFIG        : $CABOT_ROS2_RVIZ_CONFIG"
echo "CABOT_ROS2_LOCAL_RVIZ_CONFIG  : $CABOT_ROS2_LOCAL_RVIZ_CONFIG"

blue "launch gui"
rviz_option=
if [[ -n $CABOT_ROS2_RVIZ_CONFIG ]]; then
    rviz_option+=" rviz_config_file:=$CABOT_ROS2_RVIZ_CONFIG"
fi
if [[ -n $CABOT_ROS2_LOCAL_RVIZ_CONFIG ]]; then
    rviz_option+=" rviz_config_file2:=$CABOT_ROS2_LOCAL_RVIZ_CONFIG"
fi
com="$command_prefix ros2 launch -n cabot_ui cabot_gui.launch.py $rviz_option $command_postfix"
eval $com
checks+=($!)
pids+=($!)

if [[ $CABOT_GAZEBO -eq 1 ]]; then
    # Replaced with cabot_gazebo cabot_handle_simulator (2025.02.25 @daisukes)
    #
    # blue "launch cabot_keyboard teleop"
    # com="setsid xterm -e ros2 run cabot_ui cabot_keyboard.py &"
    # echo $com
    # eval $com
    # pids+=($!)
    :
else
    if [[ $CABOT_USE_HANDLE_SIMULATOR -eq 1 ]]; then
        blue "launch cabot_keyboard teleop"
        com="setsid xterm -e ros2 run cabot_ui cabot_keyboard.py &"
        echo $com
        eval $com
        pids+=($!)
    fi
fi

# launch teleop keyboard for both gazebo and physical robot
blue "launch teleop"
export ROS_LOG_DIR=$ROS_LOG_DIR/teleop_twist_keyboard
com="xterm -e ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=$CABOT_GUI_TELEOP_TWIST_TOPIC &"
echo $com
eval $com
pids+=($!)

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
