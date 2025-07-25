#!/bin/bash

# Copyright (c) 2021  IBM Corporation
# Copyright (c) 2024  Miraikan - The National Museum of Emerging Science and Innovation
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

start=`date +%s.%N`

trap ctrl_c INT QUIT TERM

terminating=0
launched=0

function ctrl_c() {
    red "catch the signal"
    user=$1
    terminating=1
    cd $scriptdir
    if [[ ! -z $dccom ]]; then
        while [[ $launched -lt 5 ]]; do
            snore 1
            launched=$((launched+1))
        done

        red "kill -INT $dcpid"
        kill -INT $dcpid
        while kill -0 $dcpid 2> /dev/null; do
            snore 1
        done
        red "$dccom down"
        if [ $verbose -eq 1 ]; then
            $dccom down
        else
            $dccom down > /dev/null 2>&1
        fi
    fi
    if [[ $run_test -eq 1 ]]; then
        # not sure but record_system_stat.launch.xml cannot
        # terminate child processes when running with run_test
        pkill -f "python3.*command_logger.py.*"
    fi
    exit $user
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
function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

function help()
{
    echo "Usage:"
    echo "-h          show this help"
    echo "-A          find all test module under cabot_sites"
    echo "-E 1-10     separate environment (ROS_DOMAIN_ID, CABOT_MAP_SERVER_HOST) for simultaneous launch"
    echo "-H          headless"
    echo "-M          log dmesg output"
    echo "-n <name>   set log name prefix"
    echo "-d          development"
    echo "-s          simulation mode"
    echo "-S <site>   override CABOT_SITE"
    echo "-t          run test"
    echo "-i <title>  specify the title for the launch_metadata yaml file"
    echo "-y          do not confirm (deprecated - always launch server if there is no server)"
    echo "-u <options> unittest"
    echo "-v          verbose option"
    echo ""
    echo "run test (-t) options"
    echo "  -D                debug"
    echo "  -f <test_regex>   run test matched with <test_regex> in CABOT_SITE.<module>"
    echo "  -L                list test modules"
    echo "  -l                list test functions"
    echo "  -r                retry test when segmentation fault"
    echo "  -T <module>       specify test module CABOT_SITE.<module> (default=tests)"
}


simulation=0
log_prefix=cabot
verbose=0
debug=0
log_dmesg=0
yes=0
run_test=0
title=""
module=tests
test_regex=
unittest=
retryoption=
list_modules=0
list_functions=0
environment=
profile=prod

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
source $scriptdir/.env

if [ "$CABOT_LAUNCH_DEV_PROFILE" == "1" ]; then
    profile=dev
fi
if [ -n "$CABOT_LAUNCH_LOG_PREFIX" ]; then
    log_prefix=$CABOT_LAUNCH_LOG_PREFIX
fi

while getopts "hDE:f:HlLMn:drsS:ti:T:uvy" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        d)
            profile=dev
            ;;
        D)
            debug=1
            ;;
        E)
            environment=$OPTARG
            ;;
        f)
            test_regex=$OPTARG
            ;;
        H)
            export CABOT_HEADLESS=1
            export USE_GUI=0
            ;;
        l)
            list_functions=1
            ;;
        L)
            list_modules=1
            ;;
        M)
            log_dmesg=1
            ;;
        n)
            log_prefix=$OPTARG
            ;;
        r)
            retryoption="-r"
            ;;
        s)
            simulation=1
            ;;
        S)
            export CABOT_SITE=$OPTARG  # override the default
            ;;
        t)
            run_test=1
            ;;
        i)
            title=$OPTARG
            ;;
        T)
            module=$OPTARG
            ;;
        u)
            unittest=1
            ;;
        v)
            verbose=1
            ;;
        y)
            yes=1
            ;;
    esac
done
shift $((OPTIND-1))

## private variables

if [[ $unittest -eq 1 ]]; then
    code=0
    docker compose run --rm navigation ./script/unittest.sh $@
    if [[ $? -ne 0 ]]; then
        code=1
    fi
    docker compose run --rm localization ./script/unittest.sh $@
    if [[ $? -ne 0 ]]; then
        code=1
    fi
    exit $code
fi

## check required environment variables
error=0
if [ -z $CABOT_MODEL ]; then
    err "CABOT_MODEL: environment variable should be specified (ex. cabot2-gt1"
    error=1
fi
if [ -z $CABOT_SITE ]; then
    err "CABOT_SITE : environment variable should be specified (ex. cabot_site_cmu_3d"
    error=1
else
    base=cabot_site_pkg
    if [[ $profile == "dev" ]]; then
        base=cabot_sites
    fi
    if find $scriptdir/$base -name $CABOT_SITE > /dev/null; then
        blue "CABOT_SITE: $CABOT_SITE exists in $base"
    else
        err "CABOT_SITE: $CABOT_SITE does not exist in $base"
        error=1
    fi
fi

if [ $error -eq 1 ]; then
   exit 1
fi

if [[ $list_modules -eq 1 ]]; then
    docker compose run --rm navigation-$profile /home/developer/ros2_ws/script/run_test.sh -L
    exit
fi

if [[ $list_functions -eq 1 ]]; then
    docker compose run --rm navigation-$profile /home/developer/ros2_ws/script/run_test.sh -l $module
    exit
fi

launch_prefix=$(basename $scriptdir)
if [[ -n $environment ]]; then
    export ROS_DOMAIN_ID=$((20+environment))
    export CABOT_MAP_SERVER_HOST="localhost:$((9090+environment*10))/map"
    export CABOT_ROSBRIDGE_PORT=$((9090+environment*10+1))
    export GAZEBO_MASTER_URI="http://127.0.0.1:$((11345+environment))"
    launch_prefix="${launch_prefix}-env${environment}"
    ./build-docker.sh -p -i -P $launch_prefix
fi

log_name=${log_prefix}_`date +%Y-%m-%d-%H-%M-%S`
export ROS_LOG_DIR="/home/developer/.ros/log/${log_name}"
export ROS_LOG_DIR_ROOT="/root/.ros/log/${log_name}"
export CABOT_LOG_NAME=$log_name
host_ros_log=$scriptdir/docker/home/.ros/log
host_ros_log_dir=$host_ros_log/$log_name
ln -snf $host_ros_log_dir $host_ros_log/latest
blue "log dir is : $host_ros_log_dir"
mkdir -p $host_ros_log_dir
cp $scriptdir/.env $host_ros_log_dir/env-file

## output launch_metadata yaml file
./script/output_launch_metadata.sh -i ${title:-$log_name} -o $host_ros_log_dir

## if network interface name for Cyclone DDS is not specified, set autoselect as true
if [ ! -z $CYCLONEDDS_URI ]; then
    if [ ! -z $CYCLONEDDS_NETWORK_INTERFACE_NAME ]; then
        export CYCLONEDDS_NETWORK_INTERFACE_AUTODETERMINE="false"
    else
        export CYCLONEDDS_NETWORK_INTERFACE_AUTODETERMINE="true"
    fi
fi

## start logging dmesg after host_ros_log_dir is defined
if [[ $log_dmesg -eq 1 ]]; then
    blue "Logging dmesg"
    dmesg --time-format iso -w > $host_ros_log_dir/dmesg.log &
fi

if [[ $terminating -eq 1 ]]; then
    exit
fi

## launch docker compose
cd $scriptdir
dcfile=

dcfile=docker-compose
if [[ $simulation -eq 0 ]]; then dcfile="${dcfile}-production"; fi
dcfile="${dcfile}.yaml"

if [ ! -e $dcfile ]; then
    err "There is not $dcfile, simulation=$simulation)"
    exit
fi

dccom="docker compose -f $dcfile -p $launch_prefix --profile $profile"

## launch server
if [[ $profile == "dev" ]]; then
    com="./server-launch.sh -d -c -p ${CABOT_SITE} -E \"$environment\""
else
    com="./server-launch.sh -c -p ${CABOT_SITE} -E \"$environment\""
fi
echo $com
eval $com

if [ $verbose -eq 0 ]; then
    com2="$dccom --ansi never up --no-build --abort-on-container-exit > $host_ros_log_dir/docker-compose.log &"
else
    com2="$dccom up --no-build --abort-on-container-exit | tee $host_ros_log_dir/docker-compose.log &"
fi
if [ $verbose -eq 1 ]; then
    blue "$com2"
fi

if [[ $terminating -eq 1 ]]; then
    exit
fi

eval $com2
dcpid=($!)
blue "[$dcpid] $dccom up $( echo "$(date +%s.%N) - $start" | bc -l )"


while [[ $launched -lt 5 ]]; do
    snore 1
    launched=$((launched+1))
done

blue "All launched: $( echo "$(date +%s.%N) - $start" | bc -l )"

if [[ $run_test -eq 1 ]]; then
    blue "Running test $module $test_regex"
    nav_service="navigation-$profile"
    if [[ $debug -eq 1 ]]; then
        docker compose -p $launch_prefix -f docker-compose-debug.yaml run debug /home/developer/ros2_ws/script/run_test.sh -w -d $module $test_regex $retryoption # debug
    else
        docker compose -p $launch_prefix -f $dcfile exec $nav_service /home/developer/ros2_ws/script/run_test.sh -w $module $test_regex $retryoption
    fi
    ctrl_c $?
fi

while [ 1 -eq 1 ];
do
    # check if any of container got Exit status
    if [[ $terminating -eq 0 ]] && [[ `$dccom ps | grep Exit | wc -l` -gt 0 ]]; then
        red "docker compose may have some issues. Check errors in the log or run with '-v' option."
        ctrl_c 1
        exit
    fi
    snore 1
done
