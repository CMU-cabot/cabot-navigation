#!/bin/bash

function help()
{
    echo "Usage:"
    echo "-h           show this help"
    echo "-p <pacakge> package for test"
    echo "-a           all"
}

package=
all=

while getopts "hp:a" arg; do
    case $arg in
	h)
	    help
	    ;;
        p)
            package=$OPTARG
            ;;
	a)
	    all=1
	    ;;
    esac
done
shift $((OPTIND-1))

if [[ -z $package ]] && [[ -z $all ]]; then
    help
    exit 1
fi

if [[ $all -eq 1 ]]; then
    colcon build
    colcon test
    colcon test-result --verbose
else
    colcon build --packages-up-to $package
    colcon test --packages-select $package
    colcon test-result --verbose --test-result-base build/$pacakge
fi
