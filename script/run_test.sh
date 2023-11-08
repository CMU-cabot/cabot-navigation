#!/bin/bash

trap ctrl_c INT QUIT TERM

function ctrl_c() {
    echo "Test terminated"
    exit 0
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
    echo "-d          log directory"
}

blue "Start runnint tests"

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

test_dir=$(find $scriptdir/../src/cabot_sites -name $CABOT_SITE | head -n 1)
tests=$test_dir/test/tests.yaml

while getopts "hd:" arg; do
    case $arg in
        h)
            help
            exit
            ;;
    esac
done

blue "testing with $CABOT_SITE"

source $scriptdir/../install/setup.bash

ros2 run cabot_navigation2 run_test.py -f $tests
result=$?

echo "Test has completed"
if [[ $result -eq 0 ]]; then
    blue "Test succeeded"
    ctrl_c 0
else
    blue "Test failed"
    ctrl_c 1
fi
