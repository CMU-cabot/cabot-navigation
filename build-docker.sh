#!/bin/bash

# Copyright (c) 2020, 2023  Carnegie Mellon University, IBM Corporation, and others
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    exit
}

function red {
    echo -en "\033[31m"  ## red
    echo $1
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $1
    echo -en "\033[0m"  ## reset color
}
function help {
    echo "Usage: $0 <option>"
    echo "$0 [<option>] [<target>]"
    echo ""
    echo "targets : all: all targets"
    echo "          ros2        : build ROS2"
    echo "          localization: build localization"
    echo "          server      : build server"
    echo "          see bellow if targets is not specified"
    echo ""
    echo "    default target=\"all\""
    echo ""
    echo "-h                    show this help"
    echo "-t <time_zone>        set time zone"
    echo "-n                    no cache option to build docker image"
    echo "-w                    build workspace only"
    echo "-i                    build image only"
    echo "-y                    no confirmation"
    echo "-u <uid>              replace uid"
    echo "-p                    prebuild"
}

ROS2_UBUNTUV=22.04
ROS2_UBUNTU_DISTRO=jammy
ROS2_DISTRO=humble

time_zone=$(cat /etc/timezone)

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)
prefix=$(basename $scriptdir)
prefix_pb=${prefix}_
build_dir=$scriptdir/cabot-common/docker

option="--progress=auto"
debug=0
build_ws=1
build_img=1
confirmation=1
prebuild=0

export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

while getopts "ht:ndwiyp" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	t)
	    time_zone=$OPTARG
	    ;;
	n)
	    option="$option --no-cache"
	    ;;
	d)
	    debug=1
	    export DOCKER_BUILDKIT=0
	    ;;
	w)
	    build_ws=1
	    build_img=0
	    ;;
	i)
	    build_ws=0
	    build_img=1
	    ;;
	y)
	    confirmation=0
	    ;;
	p)
	    prebuild=1
	    ;;
    esac
done
shift $((OPTIND-1))
targets=$@

#
# specify default targets if targets is not specified
# if targets include all set all targets
#
if [[ -z "$targets" ]] || [[ "$targets" =~ "all" ]]; then
    targets="ros2 localization server"
fi

function build_ros_base_image {
    local FROM_IMAGE=$1
    local IMAGE_TAG_PREFIX=$2
    local UBUNTU_DISTRO=$3
    local ROS_DISTRO=$4
    local ROS_COMPONENT=$5
    local -n IMAGE_TAG=$6

    echo ""
    IMAGE_TAG=$IMAGE_TAG_PREFIX-$ROS_DISTRO
    blue "## build $IMAGE_TAG"
    pushd $build_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/ros-core/

    sed s=FROM.*=FROM\ $FROM_IMAGE= Dockerfile > Dockerfile.temp && \
        docker build -f Dockerfile.temp -t $IMAGE_TAG $option .
    if [ $? -ne 0 ]; then
        red "failed to build $IMAGE_TAG"
        exit 1
    fi
    popd

    echo ""
    FROM_IMAGE=$IMAGE_TAG
    IMAGE_TAG=$IMAGE_TAG_PREFIX-$ROS_DISTRO-base
    blue "## build $IMAGE_TAG"
    pushd $build_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/ros-base/
    sed s=FROM.*=FROM\ $FROM_IMAGE= Dockerfile > Dockerfile.temp && \
        docker build -f Dockerfile.temp -t $IMAGE_TAG $option .
    if [ $? -ne 0 ]; then
        red "failed to build $IMAGE_TAG"
        exit 1
    fi
    popd

    if [[ $ROS_COMPONENT = "ros-base" ]]; then
	returnn
    fi

    echo ""
    IMAGE_TAG=$IMAGE_TAG_PREFIX-$ROS_DISTRO-desktop
    blue "## build $IMAGE_TAG"
    pushd $build_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/desktop/
    sed s=FROM.*=FROM\ $FROM_IMAGE= Dockerfile > Dockerfile.temp && \
        docker build -f Dockerfile.temp -t $IMAGE_TAG $option .
    if [ $? -ne 0 ]; then
        red "failed to build $IMAGE_TAG"
        exit 1
    fi
    popd
}

function prebuild {
    local FROM_IMAGE=$1
    local IMAGE_BASE_NAME=$2
    local IMAGE_DIR=$3
    local -n IMAGE_TAG=$4         # output variable name

    IMAGE_TAG=$IMAGE_BASE_NAME-$(basename $IMAGE_DIR)

    pushd $IMAGE_DIR
    blue "## build $IMAGE_TAG"
    docker build -t $IMAGE_TAG \
	   --file Dockerfile \
	   --build-arg TZ=$time_zone \
	   --build-arg FROM_IMAGE=$FROM_IMAGE \
	   . && popd
}

function prebuild_ros2 {
    blue "- UBUNTU_DISTRO=$ROS2_UBUNTU_DISTRO"
    blue "- ROS2_DISTRO=$ROS2_DISTRO"
    blue "- TIME_ZONE=$time_zone"

    base_image=ubuntu:$ROS2_UBUNTU_DISTRO
    base_name=${prefix_pb}_${ROS2_UBUNTU_DISTRO}
    image_tag=$base_image
    build_ros_base_image $image_tag $image_tag $ROS2_UBUNTU_DISTRO $ROS2_DISTRO desktop image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $name1"
	exit 1
    fi

    prebuild $image_tag $base_name $build_dir/${ROS2_DISTRO}-custom image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $image_tag"
	return 1
    fi

    prebuild $image_tag $image_tag $build_dir/mesa image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $image_tag"
	return 1
    fi
}


function build_ros2_ws {
    debug_option=
    if [ $debug -eq 1 ]; then
	debug_option='-d'
    fi
    docker compose run --rm navigation /launch.sh build $debug_option
}

function build_localization_ws {
    docker compose run --rm localization /launch.sh build
}

function build_server_ws {
    : # nop
}

function build_ros2_image {
    local image=${prefix_pb}_jammy-humble-custom-mesa
    docker compose build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   navigation gazebo gui
}

function build_localization_image {
    local image=${prefix_pb}_jammy-humble-custom-mesa
    docker compose build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg ROS_DISTRO=humble \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   localization
}

function build_server_image {
    docker compose  -f docker-compose-server.yaml build  \
		   $option \
		   map_server
}

blue "Targets: $targets"

if [[ $prebuild -eq 1 ]]; then
    prebuild_ros2
    if [ $? != 0 ]; then
	red "Got an error to prebuild ros2 base"
	exit 1
    fi
fi

for target in $targets; do
    if [ $build_img -eq 1 ]; then
	blue "# Building $target image"
	eval "build_${target}_image"
	if [ $? != 0 ]; then
	    red "Got an error to build $target image"
	    echo "If you want to build image run ./prebuild-docker.sh first"
	    echo "If you want to pull image run ./manage-docker-image.sh (see README) and then run ./build-docker.sh with '-w' option"
	    exit 1
	fi
    fi
    if [ $build_ws -eq 1 ]; then
	blue "# Building $target workspace"
	eval "build_${target}_ws"
	if [ $? != 0 ]; then
	    red "Got an error to build $target workspace"
	    exit 1
	fi
    fi
done
