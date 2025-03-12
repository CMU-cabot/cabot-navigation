#!/bin/bash

# Copyright (c) 2025  Carnegie Mellon University
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

trap ctrl_c INT QUIT TERM

function ctrl_c() {
    red "catch the signal"
    exit
}
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
function help()
{
    echo "Usage:"
    echo "-h          show this help"
    echo "-d          use dev profile"
    echo "-D          debug mode"
    echo "-l <lang>   language (en, zh, ja, ko)"
    echo "-t <tour>   tour id"
    echo "-s <start>  start node id"
    echo "-g <goal>   goal node id"
}

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
source $scriptdir/.env

profile=prod
debug=
lang=
tour_id=
start_id=
goal_id=

while getopts "hdDl:t:s:g:" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        d)
            profile=dev
            ;;
        D)
            debug="-d"
            ;;
        l)
            lang="-l $OPTARG"
            ;;
        t)
            tour_id="-t $OPTARG"
            ;;
        s)
            start_id="-s $OPTARG"
            ;;
        g)
            goal_id="-g $OPTARG"
            ;;
        *)
            err "Invalid option - $arg"
            help
            ;;
    esac
done
shift $((OPTIND - 1))

if [[ $CABOT_LAUNCH_DEV_PROFILE -eq 1 ]]; then
    profile=dev
fi

dccom="docker compose -f $scriptdir/docker-compose.yaml --profile $profile"
com="$dccom run --rm navigation-$profile bash -c \
     \"source install/setup.bash; script/tour_test.py $debug $lang $tour_id $start_id $goal_id\""
if [[ $debug -eq 1 ]]; then
    echo $com
fi
eval $com
