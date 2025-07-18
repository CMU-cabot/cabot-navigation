#!/bin/bash

# Copyright (c) 2020  Carnegie Mellon University
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

ulimit -S -c 0

while getopts "c" arg; do
    case $arg in
        c)
            ulimit -c unlimited
            echo 1 | sudo tee /proc/sys/kernel/core_uses_pid
            if [[ -n $ROS_LOG_DIR ]]; then
                echo "$ROS_LOG_DIR/ros2-core" | sudo tee /proc/sys/kernel/core_pattern
            else
                echo "/home/developer/ros2-core" | sudo tee /proc/sys/kernel/core_pattern
            fi
            ulimit -s 65536
            ;;
    esac
done
shift $((OPTIND-1))

if [[ $1 == "navigation" ]]; then
    shift 1
    exec ./script/cabot_ros2.sh $@
elif [[ $1 == "build" ]]; then
    shift 1
    exec ./script/cabot_build.sh $@
elif [[ $1 == "gazebo" ]]; then
    shift 1
    exec ./script/cabot_gazebo.sh $@
elif [[ $1 == "gui" ]]; then
    shift 1
    exec ./script/cabot_gui.sh $@
elif [[ $1 == "play" ]]; then
    shift 1
    exec ./script/play_bag.sh $@
elif [[ $1 == "record" ]]; then
    shift 1
    exec ./script/record_bag.sh $@
fi

exec bash
