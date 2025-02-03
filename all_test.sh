#!/bin/bash

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

trap ctrl_c INT QUIT TERM

terminating=0
launched=0

function ctrl_c() {
    red "catch the signal"
    exit
}

# check if the module is in the exclude list
function exclude() {
    local name=$1
    local module
    exclude_modules=(
	cabot_site_large_room:tests
	cabot_site_large_room:tests_crowd
	cabot_site_test_room:tests-queue
	cabot_site_test_room:tests-map-switch
    )
    
    for module in "${exclude_modules[@]}"; do
	if [[ "$module" == "$name" ]]; then
	    red "matched with excluding list : ${module}"
	    return 0
	fi
    done
    return 1
}

list_functions=0
while getopts "hl" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        l)
            list_functions=1
            ;;
    esac
done

packages=$(find cabot_sites -name package.xml)
result=0
test_results=""
for package in $packages; do
    package_path="${package%/*}"
    package_name="${package_path##*/}"
    module_path="${package_path}/${package_name}"

    if [[ -e $module_path ]]; then
	modules=$(find $module_path -name "*.py" -not -path "*__init__.py")
	for module in $modules; do
	    test_module_path=${module%.*}
	    test_module=$(basename $test_module_path)
	    if exclude "${package_name}:${test_module}"; then
		continue
	    fi
	    blue "Test module: ${package_name}/${test_module}"
	    if [[ $list_functions -eq 1 ]]; then
		./launch.sh -s -t -H -S $package_name -T $test_module -l
	    else
		./launch.sh -s -t -H -S $package_name -T $test_module
		temp=$?
		result=$((result & temp))		
		if [[ $temp -eq 0 ]]; then
		    test_results=$test_results"\033[32mTest $package_name/$test_module Success\033[0m\n"
		else
		    test_results=$test_results"\033[31mTest $package_name/$test_module Fail\033[0m\n"
		fi
		echo -en $test_results
		sleep 5
	    fi
	done
    fi
done

echo -en $test_results
