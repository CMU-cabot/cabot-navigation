#!/bin/bash

function help()
{
    echo "Usage:"
    echo "-h           show this help"
    echo "-p <pacakge> package for test"
    echo "-a           all"
    echo "-b           build"
}

package=
all=
build=
exclude="(rosbridge.*|rosapi)"

while getopts "hp:ab" arg; do
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
        b)
            build=1
            ;;
    esac
done
shift $((OPTIND-1))

if [[ -z $package ]] && [[ -z $all ]]; then
    help
    exit 1
fi

if [[ $all -eq 1 ]]; then
    ([[ -z $build ]] ||  colcon build) && \
    colcon test --packages-ignore-regex "$exclude" && \
    colcon test-result --verbose
else
    ([[ -z $build ]] ||  colcon build --packages-up-to $package) && \
    colcon test --packages-select $package && \
    colcon test-result --verbose --test-result-base build/$package
fi
