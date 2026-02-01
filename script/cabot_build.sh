#!/bin/bash

# Copyright (c) 2022  Carnegie Mellon University
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

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

if [[ -e install/setup.bash ]]; then
    source install/setup.bash
else
    source /opt/underlay_ws/install/setup.bash
fi

# Ensure colcon writes logs to a writable location in containers.
COLCON_LOG_PATH=${COLCON_LOG_PATH:-/tmp/colcon_log}
export COLCON_LOG_PATH
sudo mkdir -p "$COLCON_LOG_PATH"
sudo chown -R "$(id -u):$(id -g)" "$COLCON_LOG_PATH"

cd ..
ros2_ws=`pwd`

# Ensure workspace directories are writable (volume may be root-owned from previous builds).
for dir in "$ros2_ws/build" "$ros2_ws/install"; do
    if [[ ! -d "$dir" ]]; then
        sudo mkdir -p "$dir"
    fi
    sudo chown -R "$(id -u):$(id -g)" "$dir"
done

debug=0
sequential=0
vlm_pkgs=(vlm_social_nav vsn_yolo_ros vsn_yolo_msgs social_nav_plugin)
vlm_root="$ros2_ws/src/VLM-Social-Nav-ROS2"
vlm_src_root="$vlm_root/src"

while getopts "ds" arg; do
    case $arg in
        d)
            debug=1
            ;;
        s)
            sequential=1
            ;;
    esac
done
shift $((OPTIND-1))

if [[ -d "$vlm_src_root" ]]; then
    for pkg in "${vlm_pkgs[@]}"; do
        cache="$ros2_ws/build/$pkg/CMakeCache.txt"
        if [[ -f "$cache" ]]; then
            src_dir=$(grep -E "^${pkg}_SOURCE_DIR:" "$cache" | head -n1 | cut -d= -f2-)
            expected="$vlm_src_root/$pkg"
            if [[ -n "$src_dir" ]] && [[ "$src_dir" != "$expected" ]]; then
                echo "cleaning stale build cache for $pkg (expected $expected, got $src_dir)"
                rm -rf "$ros2_ws/build/$pkg" "$ros2_ws/install/$pkg"
            fi
        fi
    done
fi

# Force install/build into the workspace so host bind mount gets populated.
COLCON_BUILD_BASE="$ros2_ws/build"
COLCON_INSTALL_BASE="$ros2_ws/install"
build_option=" --build-base $COLCON_BUILD_BASE --install-base $COLCON_INSTALL_BASE"
if [[ $debug -eq 1 ]]; then
    build_option+=" --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install"
else
    build_option+=" --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
fi
if [[ $sequential -eq 1 ]]; then
    build_option+=" --executor sequential"
fi
if [[ -d "$vlm_src_root" ]]; then
    colcon build $build_option --cmake-clean-cache --cmake-force-configure --packages-select "${vlm_pkgs[@]}"
    if [[ $? -ne 0 ]]; then exit 1; fi
fi
colcon build $build_option "$@"

if [[ $? -ne 0 ]]; then exit 1; fi

if [[ -e $scriptdir/../src/queue_utils_py ]]; then
    cd $scriptdir/../src/queue_utils_py
    pip3 install .
fi
