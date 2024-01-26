#!/bin/bash

# Copyright (c) 2021  IBM Corporation
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

	red "$dccom down"
	if [ $verbose -eq 1 ]; then
	    $dccom down
	else
	    $dccom down > /dev/null 2>&1
	fi
    fi

    for pid in ${pids[@]}; do
        signal=2
	if [[ "${termpids[*]}" =~ "$pid" ]]; then
            signal=15
        fi
        if [ $verbose -eq 1 ]; then
            echo "killing $0 $pid"
            kill -s $signal $pid
        else
            echo "killing $0 $pid"
            kill -s $signal $pid > /dev/null 2>&1
        fi
    done
    for pid in ${pids[@]}; do
        if [ $verbose -eq 1 ]; then
            while kill -0 $pid; do
                echo "waiting $0 $pid"
                snore 1
            done
        else
            echo "waiting $0 $pid"
            while kill -0 $pid > /dev/null 2>&1; do
                snore 1
            done
        fi
    done
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
    echo "-s          simulation mode"
    echo "-p <name>   docker compose's project name"
    echo "-n <name>   set log name prefix"
    echo "-v          verbose option"
    echo "-c <name>   config name (default=) docker-compose(-<name>)(-production).yaml will use"
    echo "            if there is no nvidia-smi and config name is not set, automatically set to 'nuc'"
    echo "-3          equivalent to -c rs3"
    echo "-M          log dmesg output"
    echo "-S          record screen cast"
    echo "-y          do not confirm"
    echo "-t          run test"
    echo "-T <module> run test CABOT_SITE.<module>"
    echo "-f <test>   run test CABOT_SITE.<module>.<test>"
}


simulation=0
use_nuc=0
nvidia_gpu=0
project_option=
log_prefix=cabot
verbose=0
config_name=
local_map_server=0
debug=0
reset_all_realsence=0
log_dmesg=0
screen_recording=0
yes=0
run_test=0
module=tests
test_func=

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
source $scriptdir/.env

if [ -n "$CABOT_LAUNCH_CONFIG_NAME" ]; then
    config_name=$CABOT_LAUNCH_CONFIG_NAME
fi
if [ -n "$CABOT_LAUNCH_DO_NOT_RECORD" ]; then
    do_not_record=$CABOT_LAUNCH_DO_NOT_RECORD
fi
if [ -n "$CABOT_LAUNCH_RECORD_CAMERA" ]; then
    record_cam=$CABOT_LAUNCH_RECORD_CAMERA
fi
if [ -n "$CABOT_LAUNCH_LOG_PREFIX" ]; then
    log_prefix=$CABOT_LAUNCH_LOG_PREFIX
fi

while getopts "hsdrp:n:vc:3DMSytHT:f:" arg; do
    case $arg in
        s)
            simulation=1
            ;;
        h)
            help
            exit
            ;;
        d)
            do_not_record=1
            ;;
        r)
            record_cam=1
            ;;
        p)
            project_option="-p $OPTARG"
            ;;
        n)
            log_prefix=$OPTARG
            ;;
        v)
            verbose=1
            ;;
	c)
	    config_name=$OPTARG
	    ;;
	3)
	    config_name=rs3
	    ;;
	D)
	    debug=1
	    ;;
	M)
	    log_dmesg=1
	    ;;
	S)
	    screen_recording=1
	    ;;
	y)
	    yes=1
	    ;;
	t)
	    run_test=1
	    ;;
	T)
        module=$OPTARG
	    ;;
	f)
	    test_func=$OPTARG
	    ;;
	H)
	    export CABOT_HEADLESS=1
	    ;;
    esac
done
shift $((OPTIND-1))


## private variables
pids=()
termpids=()

## check nvidia-smi
if [ -z `which nvidia-smi` ]; then
    if [ -z $config_name ]; then
	red "[WARNING] cannot find nvidia-smi, so config_name is changed to 'nuc'"
	config_name=nuc
    fi
else
    nvidia_gpu=1
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
fi

if [ "$config_name" = "rs3" ]; then
    if [ -z $CABOT_REALSENSE_SERIAL_1 ]; then
	err "CABOT_REALSENSE_SERIAL_1: environment variable should be specified"
	error=1
    fi
    if [ -z $CABOT_REALSENSE_SERIAL_2 ]; then
	err "CABOT_REALSENSE_SERIAL_2: environment variable should be specified"
	error=1
    fi
    if [ -z $CABOT_REALSENSE_SERIAL_3 ]; then
	err "CABOT_REALSENSE_SERIAL_3: environment variable should be specified"
	error=1
    fi
    reset_all_realsence=1
fi

if [[ "$config_name" = "nuc" ]]; then
    if [[ -z $CABOT_JETSON_CONFIG ]]; then
	err "CABOT_JETSON_CONFIG: environment variable should be specified to launch people on Jetson"
	error=1
    fi
fi

if [ $error -eq 1 ]; then
   exit 1
fi

cabot_site_dir=$(find $scriptdir/cabot_sites -name $CABOT_SITE)
if [[ -e $cabot_site_dir/server_data ]]; then
    local_map_server=1
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
    termpids+=($!)
    pids+=($!)
fi

if [[ $terminating -eq 1 ]]; then
    exit
fi

## launch docker compose
cd $scriptdir
dcfile=

dcfile=docker-compose
if [ ! -z $config_name ]; then dcfile="${dcfile}-$config_name"; fi
if [ $simulation -eq 0 ]; then dcfile="${dcfile}-production"; fi
if [ $debug -eq 1 ]; then dcfile=docker-compose-debug; fi            # only basic debug
dcfile="${dcfile}.yaml"

if [ ! -e $dcfile ]; then
    err "There is not $dcfile (config_name=$config_name, simulation=$simulation)"
    exit
fi

dccom="docker compose $project_option -f $dcfile"

if [ $local_map_server -eq 1 ]; then
    blue "Checking the map server is available $( echo "$(date +%s.%N) - $start" | bc -l )"
    curl http://localhost:9090/map/map/floormaps.json --fail > /dev/null 2>&1
    test=$?
    launching_server=0
    while [[ $test -ne 0 ]]; do
	if [[ $launching_server -eq 1 ]]; then
	    snore 5
	    blue "waiting the map server is ready..."
	    curl http://localhost:9090/map/map/floormaps.json --fail > /dev/null 2>&1
	    test=$?
	else
	    if [[ $yes -eq 0 ]]; then
		red "Note: launch.sh no longer launch server in the script"
		red -n "You need to run local web server for $CABOT_SITE, do you want to launch the server [Y/N]: "
		read -r ans
	    else
		ans=y
	    fi
	    if [[ $ans = 'y' ]] || [[ $ans = 'Y' ]]; then
		launching_server=1
		gnome-terminal -- bash -c "./server-launch.sh -d $cabot_site_dir/server_data; exit"
	    else
		echo ""
		exit 1
	    fi
	fi
    done
fi

if [ $verbose -eq 0 ]; then
    com2="bash -c \"setsid $dccom --ansi never up --no-build --abort-on-container-exit\" > $host_ros_log_dir/docker-compose.log &"
else
    com2="bash -c \"setsid $dccom up --no-build --abort-on-container-exit\" | tee $host_ros_log_dir/docker-compose.log &"
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
    blue "Running test"
    docker compose exec navigation /home/developer/ros2_ws/script/run_test.sh $module $test_func
    pids+=($!)
    runtest_pid=$!
    snore 3
fi

while [ 1 -eq 1 ];
do
    # check if any of container got Exit status
    if [[ $terminating -eq 0 ]] && [[ `$dccom ps | grep Exit | wc -l` -gt 0 ]]; then
	red "docker compose may have some issues. Check errors in the log or run with '-v' option."
	ctrl_c 1
	exit
    fi
    if [[ $run_test -eq 1 ]]; then
	kill -0 $runtest_pid
	if [[ $? -eq 1 ]]; then
	    ctrl_c 1
	    exit
	fi
    fi
    snore 1
done
