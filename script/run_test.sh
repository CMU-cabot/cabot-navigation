#!/bin/bash

trap ctrl_c INT QUIT TERM

function ctrl_c() {
    echo "Test terminated"
    exit $1
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
    echo "  $0 [<options>] <module> <test function name regex>"
    echo ""
    echo "-h          show this help"
    echo "-d          debug print"
    echo "-L          list test modules"
    echo "-l          list test functions"
    echo "-r          retry until the test succeeds"
    echo "-w          wait ready"
}

blue "Start runnint tests"

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

wait_ready_option=
debug=
list_modules=0
list_functions=0
retry=0

while getopts "hdLlrw" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        d)
            debug="-d"
            ;;
	L)
	    list_modules=1
	    ;;
	l)
	    list_functions=1
	    ;;
        r)
            retry=1
            ;;
        w)
            wait_ready_option="-w"
            ;;
    esac
done
shift $((OPTIND-1))

module=$1
test_func_option=""
if [[ $2 != "" ]]; then
  test_func_option="-f $2"
fi  

output_dir_option="-o ${ROS_LOG_DIR}"

blue "testing with $CABOT_SITE"

source $scriptdir/../install/setup.bash

while [[ 1 -eq 1 ]]; do
    if [[ $list_modules -eq 1 ]]; then
	ros2 run cabot_navigation2 run_test.py -m ${CABOT_SITE} -L $output_dir_option
	exit
    fi

    if [[ $list_functions -eq 1 ]]; then
	ros2 run cabot_navigation2 run_test.py -m ${CABOT_SITE}.$module -l $output_dir_option
	exit
    fi

    if [[ ! -z $debug ]]; then
	echo "ros2 run --prefix 'gdb -ex run -ex bt -ex quit --args' cabot_navigation2 run_test.py -m ${CABOT_SITE}.$module $test_func_option $wait_ready_option $debug $output_dir_option"
	ros2 run --prefix 'gdb -ex run -ex bt -ex quit --args python3' cabot_navigation2 run_test.py -m ${CABOT_SITE}.$module $test_func_option $wait_ready_option $debug $output_dir_option
    else
	echo "ros2 run cabot_navigation2 run_test.py -m ${CABOT_SITE}.$module $test_func_option $wait_ready_option $debug $output_dir_option"
	ros2 run cabot_navigation2 run_test.py -m ${CABOT_SITE}.$module $test_func_option $wait_ready_option $output_dir_option
    fi
    result=$?
    if [[ $result -le 1 ]]; then
        break
    else
        if [[ $retry -eq 0 ]]; then
            break
        fi
        echo "retry test, due to segmantation fault"
        wait_ready_option=""
    fi
done

echo "Test has completed"
if [[ $result -eq 0 ]]; then
    blue "Test succeeded"
    ctrl_c 0
else
    blue "Test failed"
    ctrl_c 1
fi
