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

set -m

## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    blue "trap cabot_mf_localization.sh "

    for pid in ${pids[@]}; do
        com="kill -SIGINT $pid"
        red $com
        eval $com
    done
    for pid in ${pids[@]}; do
        count=0
        while kill -0 $pid 2> /dev/null; do
            if [[ $count -eq 10 ]]; then
                blue "escalate to SIGTERM $pid"
                com="kill -TERM $pid"
                eval $com
            fi
            if [[ $count -eq 20 ]]; then
                blue "escalate to SIGKILL $pid"
                com="kill -KILL $pid"
                eval $com
            fi
            echo "waiting $0 $pid"
            snore 1
            count=$((count+1))
        done
    done
    
    exit
}


## todo duplicate code
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

## private variables
pids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

### default variables

## debug
#minimum=0
debug=0
command=''
commandpost='&'

# for localization
points2_topic='/velodyne_points'
imu_topic='/cabot/imu/data'
beacons_topic='/wireless/beacons'
wifi_topic='/esp32/wifi'
odom_topic='/cabot/odometry/filtered'
pressure_topic='/cabot/pressure'
gnss_fix_topic='/ublox/fix'
gnss_fix_velocity_topic='/ublox/fix_velocity'
fix_filtered_topic='/ublox/fix_filtered'
fix_status_threshold=2
publish_current_rate=0

: ${CABOT_GAZEBO:=0}
: ${CABOT_SITE:=}
: ${CABOT_SITE_TAGS:='\"\"'}
: ${CABOT_MODEL:=}
: ${CABOT_SHOW_LOC_RVIZ:=0}
: ${CABOT_HEADLESS:=0}
if [[ $CABOT_HEADLESS -eq 1 ]]; then
    CABOT_SHOW_LOC_RVIZ=0
fi
: ${CABOT_PRESSURE_AVAILABLE:=0}
: ${CABOT_ROSBAG_COMPRESSION:='message'}
: ${CABOT_USE_GNSS:=0}
# global localizer
: ${CABOT_RUN_GLOBAL_LOCALIZER:=0}
: ${CABOT_USE_GLOBAL_LOCALIZER:=0}

# mapping
: ${MAPPING_USE_GNSS:=false}
: ${MAPPING_RESOLUTION:=}

# ntrip client parameters
: ${GNSS_NODE_START_AT_LAUNCH:=1}
: ${NTRIP_CLIENT_START_AT_LAUNCH:=0}
: ${NTRIP_CLIENT:=ntrip_client}
: ${NTRIP_HOST:=}
: ${NTRIP_PORT:=}
: ${NTRIP_MOUNTPOINT:=}
: ${NTRIP_AUTHENTIFICATE:=}
: ${NTRIP_USERNAME:=}
: ${NTRIP_STR2STR_RELAY_BACK:=0}

gazebo=$CABOT_GAZEBO
site=$CABOT_SITE
show_rviz=$CABOT_SHOW_LOC_RVIZ
robot=$CABOT_MODEL
robot_desc=$CABOT_MODEL
# set 0 to the default value so that adding -p means using pressure topic.
pressure_available=$CABOT_PRESSURE_AVAILABLE
use_gnss=$CABOT_USE_GNSS
# global localizer
run_global_localizer=$CABOT_RUN_GLOBAL_LOCALIZER
use_global_localizer=$CABOT_USE_GLOBAL_LOCALIZER

# for navigation
navigation=0
localization=1
cart_mapping=0
map_server=0
with_human=1
gplanner='base_global_planner:=navfn/NavfnROS'
lplanner='base_local_planner:=dwa_local_planner/DWAPlannerROS'
cmd_vel_topic='/cmd_vel'

### usage print function
function usage {
    echo "Usage"
    echo "    run this script after running cabot.sh in another terminal"
    echo "ex)"
    echo $0 "-O -T <site_package>"
    echo ""
    echo "-h                       show this help"
    echo "-d                       debug"
    echo "-m <map file>            specify a map file"
    echo "-n <anchor file>         specify a anchor file, use map file if not specified"
    echo "-w <world file>          specify a world file"
    echo "-s                       specify its on simulation (gazebo)"
    echo "-O                       performance (no rviz)"
    echo "-T <site package>        packge name for the robot site"
    echo "-N                       start ROS1 navigation"
    echo "-M                       start multi-floor map server"
    echo "-r <robot>               specify a robot for navigation"
    echo "-f                       use robot footprint without human for navigation"
    echo "-R <rate:float>          set publish_current_rate"
    echo "-X                       do not start localization"
    echo "-C                       run cartographer mapping"
    echo "-p                       use pressure topic for height change detection"
    echo "-G                       use gnss fix for outdoor localization"
    exit
}

while getopts "hdm:n:w:sOT:NMr:fR:XCpG" arg; do
    case $arg in
    h)
        usage
        exit
        ;;
    d)
        debug=1
        command="setsid xterm -e '"
        commandpost=";read'&"
        ;;
    m)
        map=$OPTARG
        ;;
    n)
        anchor=$OPTARG
        ;;
    w)
        world=$OPTARG
        ;;
    s)
        gazebo=1
        ;;
    O)
        show_rviz=0
        ;;
    T)
        site=$OPTARG
        ;;
    N)
        navigation=1
        ;;
    M)
        map_server=1
        ;;
    r)
        robot=$OPTARG
        robot_desc=$robot
        ;;
    f)
        with_human=0
        ;;
    R)
        publish_current_rate=$OPTARG
        ;;
    X)
        localization=0
        ;;
    C)
        cart_mapping=1
        ;;
    p)
        pressure_available=1
        ;;
    G)
        use_gnss=1
        ;;
  esac
done
shift $((OPTIND-1))


# load site data for localization/simulation
if [ $cart_mapping -eq 0 ] || [ $gazebo -eq 1 ]; then
    # if site is defined
    if [ "$site" != "" ]; then
        sitedir=`ros2 pkg prefix $site`/share/$site
        source $sitedir/config/config.sh
        if [ "$map" == "" ] && [ "$world" == "" ]; then
            echo "Please check config/config.sh in site package ($sitedir) to set map and world"
            exit
        fi
    fi
    # check map and world
    if [ "$map" == "" ]; then
            echo "-T <site> or -m <map> should be specified"
            exit
    fi
    if [ $gazebo -eq 1 ] && [ "$world" == "" ]; then
            echo "-T <site> or -w <world> should be specified"
            exit
    fi
else
    # mapping
    echo "Skip site package check in mapping mode (site=$site)"
fi


## debug output
echo "Debug         : $debug ($command, $commandpost)"
echo "World         : $world"
echo "Map           : $map"
echo "Anchor        : $anchor"
echo "Simulation    : $gazebo"
echo "Navigation    : $navigation"
echo "Map server    : $map_server"
echo "With human    : $with_human"
echo "Robot         : $robot"
echo "Global planner: $gplanner"
echo "Local planner : $lplanner"
echo "Use gnss fix  : $use_gnss"
echo "Run global localizer: $run_global_localizer"
echo "Use global localizer : $use_global_localier"

### For GAZEBO simulation, run wireless simulator with gazebo
### For physical robots, run wireless scanner separately
if [ $gazebo -eq 1 ]; then
  if [ "$wireless_config" == "" ]; then
    echo "does not launch ble_rss_simulator (world_config was not found)."
  else
    echo "launch gazebo helpers (ble_rss_simulator, floor_transition)"
    wireless_config=$(realpath $wireless_config)
    com="$command ros2 launch -n mf_localization_gazebo gazebo_helper.launch.py \
                    beacons_topic:=$beacons_topic \
                    wifi_topic:=$wifi_topic \
                    wireless_config_file:=$wireless_config \
                    $commandpost"
    echo $com
    eval $com
    pids+=($!)
    echo "${pids[@]}"
  fi
else
    # launch ntrip client and/or ublox node
    if [ ${NTRIP_CLIENT_START_AT_LAUNCH} -eq 1 ]; then
        # if NTRIP_HOST exists in environment variables, use it.
        # if not, load NTRIP_HOST from CABOT_SITE package
        if [ "${NTRIP_HOST}" = "" ]; then
            echo "NTRIP_HOST does not exist in environment variables. load NTRIP_HOST from CABOT_SITE package"
            if [ "${CABOT_SITE}" != "" ]; then
                sitedir=`ros2 pkg prefix $CABOT_SITE`/share/$CABOT_SITE
                rtk_config_file=$sitedir/config/rtk_config.sh
                if [ -e $rtk_config_file ]; then
                    source $sitedir/config/rtk_config.sh
                else
                    red "File not found: $rtk_config_file"
                fi
            fi
        fi

        # check if NTRIP_HOST is defined.
        if [ "${NTRIP_HOST}" = "" ]; then
            while [ 1 -eq 1 ]
            do
            red "You need to specify NTRIP_HOST or CABOT_SITE environment variable"
            snore 1
            done
        fi
    fi

    # ntrip client
    ntrip_client_arg=""
    if [ ${NTRIP_CLIENT_START_AT_LAUNCH} -eq 1 ]; then
        if [ "${NTRIP_CLIENT}" = "str2str_node" ]; then
            # launch str2str_node
            ntrip_client_arg="str2str_node:=true"
        elif [ "${NTRIP_CLIENT}" = "ntrip_client" ]; then
            # launch ntrip_client
            ntrip_client_arg="ntrip_client:=true"
        fi
    else
        if [ "${NTRIP_CLIENT}" = "str2str_node" ]; then
            # launch str2str_node
            ntrip_client_arg="str2str_node_logger:=true"
        elif [ "${NTRIP_CLIENT}" = "ntrip_client" ]; then
            # launch ntrip_client
            ntrip_client_arg="ntrip_client_logger:=true"
        fi
    fi

    # gnss node
    gnss_arg=""
    if [ ${GNSS_NODE_START_AT_LAUNCH} -eq 1 ]; then
        gnss_arg="ublox_node:=true"
    else
        gnss_arg="ublox_node_logger:=true"
    fi
    # change error level of NO FIX status when GNSS is not used in localization
    if [ ${CABOT_USE_GNSS} -eq 0 ]; then
        gnss_arg="$gnss_arg fix_warn_error_level:=0 no_fix_error_level:=0"
    fi

    # gnss.launch.py command
    cmd=""
    if [ ${NTRIP_CLIENT_START_AT_LAUNCH} -eq 1 ]; then
        cmd="$command ros2 launch -n mf_localization gnss.launch.py \
                $ntrip_client_arg \
                $gnss_arg \
                host:=$NTRIP_HOST \
                port:=$NTRIP_PORT \
                mountpoint:=$NTRIP_MOUNTPOINT \
                authentificate:=$NTRIP_AUTHENTIFICATE \
                username:=$NTRIP_USERNAME \
                password:=$NTRIP_PASSWORD \
                relay_back:=$NTRIP_STR2STR_RELAY_BACK \
                $commandpost"
    else
        cmd="$command ros2 launch -n mf_localization gnss.launch.py \
                $ntrip_client_arg \
                $gnss_arg \
                $commandpost"
    fi
    echo $cmd
    eval $cmd
    pids+=($!)
fi

gazebo_bool=$([[ $gazebo -eq 1 ]] && echo 'true' || echo 'false')\

### launch rviz
if [ $show_rviz -eq 1 ]; then
   echo "launch rviz"
   cmd="$command ros2 launch -n mf_localization view_multi_floor.launch.py \
         use_sim_time:=$gazebo_bool $commandpost"
   echo $cmd
   eval $cmd
   pids+=($!)
fi

# mapping
if [ $cart_mapping -eq 1 ]; then
    if [[ $gazebo -eq 0 ]]; then
        imu_topic=/imu/data
    fi

    # switch lidar if specified
    : ${LIDAR_MODEL:=}
    echo "LIDAR_MODEL=$LIDAR_MODEL"
    if [ "${LIDAR_MODEL}" != "VLP16" ]; then
        USE_VELODYNE=false
        if [ "${LIDAR_MODEL}" = "XT32" ] || [ "${LIDAR_MODEL}" = "XT16" ]; then
            if [ "${LIDAR_MODEL}" = "XT32" ]; then
                model_for_lidar=cabot3-m1
            elif [ "${LIDAR_MODEL}" = "XT16" ]; then
                model_for_lidar=cabot3-m2
            fi
            echo "launch node for $LIDAR_MODEL"
            cmd="$command ros2 launch -n cabot_base hesai_lidar.launch.py \
                model:=$model_for_lidar \
                pandar:=/velodyne_points \
                $commandpost"
            echo $cmd
            eval $cmd
            pids+=($!)
        elif [ "${LIDAR_MODEL}" = "" ]; then
            echo "LIDAR_MODEL is not set. Skip starting lidar driver"
            imu_topic=/cabot/imu/data
        else
            echo "Please specify a known lidar model (LIDAR_MODEL=$LIDAR_MODEL)"
            exit
        fi
    fi

    # define and switch record_points variable
    record_points=false  # VLP-16
    if [ $gazebo -eq 1 ]; then
        record_points=true
    elif [ "${LIDAR_MODEL}" != "VLP16" ]; then
        record_points=true
    fi

    # find robot description from cabot_description package
    robot_desc_pkg_share_dir=`ros2 pkg prefix --share cabot_description`
    robot_xacro=$robot_desc_pkg_share_dir/robots/$robot.urdf.xacro.xml
    cabot_model=""
    # if robot_xacro is not found,
    if [ -e $robot_xacro ]; then
        echo "robot xacro found. use cabot_model:=$robot. (robot_xacro=$robot_xacro)"
        cabot_model=$robot
    else
        red "robot xacro NOT found. use robot:=$robot instead. (robot_xacro=$robot_xacro)"
        cabot_model=""
    fi

    # show mapping variables
    echo "MAPPING_USE_GNSS=$MAPPING_USE_GNSS"
    echo "MAPPING_RESOLUTION=$MAPPING_RESOLUTION"

    # pre process config file
    bag_filename=${OUTPUT_PREFIX}_`date +%Y-%m-%d-%H-%M-%S`
    work_dir=/home/developer/recordings
    pkg_share_dir=`ros2 pkg prefix --share mf_localization_mapping`

    # copy config file to a temp directory
    configuration_directory=$pkg_share_dir/configuration_files/cartographer
    configuration_directory_tmp=$configuration_directory/tmp
    mkdir -p $configuration_directory_tmp
    cp $configuration_directory/cartographer_2d_mapping.lua $configuration_directory_tmp

    # read grid size from config file or update grid size with the environment variable
    if [ "${MAPPING_RESOLUTION}" = "" ]; then
        MAPPING_RESOLUTION=$(grep '^TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution' $configuration_directory/cartographer_2d_mapping.lua | sed -E 's/.*= ([0-9.]+).*/\1/')
        if [ "${MAPPING_RESOLUTION}" != "" ]; then
            echo "Read grid resolution from cartographer_2d_mapping.lua (grid_size=$MAPPING_RESOLUTION)"
        else
            MAPPING_RESOLUTION=0.05
            echo "Failed to read grid resolution from cartographer_2d_mapping.lua. Use default grid_size=$MAPPING_RESOLUTION"
        fi
    else
        sed -i 's/return options/TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = '$MAPPING_RESOLUTION'\
return options/g' $configuration_directory_tmp/cartographer_2d_mapping.lua
        echo "Update cartographer_2d_mapping.lua with grid_size=$MAPPING_RESOLUTION"
    fi

    # backup config file
    cp $configuration_directory_tmp/cartographer_2d_mapping.lua \
          $work_dir/${bag_filename}.cartographer_2d_mapping.lua
    cp $pkg_share_dir/configuration_files/mapping_config.yaml \
          $work_dir/${bag_filename}.mapping_config.yaml

    # build cmd
    cmd="$command"
    cmd="$cmd ros2 launch -n mf_localization_mapping realtime_cartographer_2d_VLP16.launch.py \
          run_cartographer:=${RUN_CARTOGRAPHER:-true} \
          record_wireless:=true \
          save_samples:=true \
          record_required:=true \
          record_points:=$record_points \
          compression_mode:=$CABOT_ROSBAG_COMPRESSION \
          use_xsens:=${USE_XSENS:-true} \
          use_arduino:=${USE_ARDUINO:-false} \
          use_esp32:=${USE_ESP32:-false} \
          use_velodyne:=${USE_VELODYNE:-true} \
          imu_topic:=${imu_topic} \
          use_sim_time:=$gazebo_bool \
          grid_resolution:=${MAPPING_RESOLUTION} \
          configuration_directory:=$configuration_directory_tmp \
          save_pose:=true \
          save_trajectory:=true \
          interpolate_samples_by_trajectory:=true \
          bag_filename:=$bag_filename"
    if [ $cabot_model != "" ]; then
        cmd="$cmd cabot_model:=$cabot_model"
    else
        cmd="$cmd robot:=$robot"
    fi

    if [ "$MAPPING_USE_GNSS" = true ]; then
        # backup config file
        cp $configuration_directory/cartographer_2d_mapping_gnss.lua $configuration_directory_tmp
        cp $configuration_directory_tmp/cartographer_2d_mapping_gnss.lua \
              $work_dir/${bag_filename}.cartographer_2d_mapping_gnss.lua

        cmd="$cmd \
            fix_topic:=$fix_filtered_topic \
            configuration_basename:=cartographer_2d_mapping_gnss.lua \
            mapping_use_gnss:=true \
            "
    fi

    cmd="$cmd $commandpost"
    echo $cmd
    eval $cmd
    pids+=($!)
fi

if [ $localization -eq 0 ]; then
    while [ 1 -eq 1 ];
    do
        snore 1
    done
    exit
fi

### launch localization
if [ $navigation -eq 0 ]; then
    # run mf_localization only
    launch_file="multi_floor_2d_rss_localization.launch.py"
    echo "launch $launch_file"
    com="$command ros2 launch -n mf_localization $launch_file \
                    robot:=$robot \
                    map_config_file:=$map \
                    tags:=$CABOT_SITE_TAGS \
                    with_odom_topic:=true \
                    beacons_topic:=$beacons_topic \
                    wifi_topic:=$wifi_topic \
                    points2_topic:=$points2_topic \
                    imu_topic:=$imu_topic \
                    odom_topic:=$odom_topic \
                    pressure_available:=$([[ $pressure_available -eq 1 ]] && echo 'true' || echo 'false') \
                    pressure_topic:=$pressure_topic \
                    use_gnss:=$([[ $use_gnss -eq 1 ]] && echo 'true' || echo 'false') \
                    gnss_fix_topic:=$gnss_fix_topic \
                    gnss_fix_velocity_topic:=$gnss_fix_velocity_topic \
                    run_global_localizer:=$([[ $run_global_localizer -eq 1 ]] && echo 'true' || echo 'false') \
                    use_global_localizer:=$([[ $use_global_localizer -eq 1 ]] && echo 'true' || echo 'false') \
                    publish_current_rate:=$publish_current_rate \
                    use_sim_time:=$gazebo_bool \
                    $commandpost"
    echo $com
    eval $com
    pids+=($!)
    echo "${pids[@]}"

    # (daisueks) this is not required with ROS2
    # launch multi-floor map server for visualization
#    if [ $map_server -eq 1 ]; then
#        echo "launch multi_floor_map_server.launch"
#        com="$command ros2 launch -n mf_localization multi_floor_map_server.launch.xml \
#                        map_config_file:=$map \
#                        $commandpost"
#	echo $com
#	eval $com
#        pids+=($!)
#    fi
else
    # run navigation (mf_localization + planning)
    echo "launch multicart_demo.launch"
    com="$command ros2 launch -n cabot_mf_localization multicart_demo.launch.py \
                    map_file:=$map \
                    beacons_topic:=$beacons_topic \
                    points2_topic:=$points2_topic \
                    imu_topic:=$imu_topic \
                    odom_topic:=$odom_topic \
                    publish_current_rate:=$publish_current_rate \
                    cmd_vel_topic:=$cmd_vel_topic \
                    $lplanner $gplanner \
                    robot:=$robot \
                    with_human:=$with_human \
                    $commandpost"
    echo $com
    eval $com
    pids+=($!)
fi

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    snore 1
done
