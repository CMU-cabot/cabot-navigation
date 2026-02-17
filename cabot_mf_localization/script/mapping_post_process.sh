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

robot=$CABOT_MODEL
WORKDIR=/home/developer/post_process
QUIT_WHEN_ROSBAG_FINISH=${QUIT_WHEN_ROSBAG_FINISH:-true}
PLAYBAG_RATE_CARTOGRAPHER=${PLAYBAG_RATE_CARTOGRAPHER:-1.0}
PLAYBAG_RATE_PC2_CONVERT=${PLAYBAG_RATE_PC2_CONVERT:-1.0}
: ${LIDAR_MODEL:=}
MAPPING_USE_GNSS=${MAPPING_USE_GNSS:-false}
: ${MAPPING_RESOLUTION:=}
CONVERT_BAG=${CONVERT_BAG:-true}

gazebo=${PROCESS_GAZEBO_MAPPING:-0}

# topic
points2_topic='velodyne_points'
imu_topic=imu/data
fix_topic=ublox/fix
fix_velocity_topic=ublox/fix_velocity
fix_filtered_topic=ublox/fix_filtered
fix_status_threshold=2
convert_points=true  # VLP16 (default LIDAR_MODEL)

if [[ $gazebo -eq 1 ]]; then
    imu_topic=cabot/imu/data
    convert_points=false
    fix_status_threshold=0
elif [ "${LIDAR_MODEL}" != "VLP16" ]; then
    convert_points=false
    if [ "${LIDAR_MODEL}" = "" ]; then
      imu_topic=cabot/imu/data
    fi
fi

echo "mapping post process"
echo "CABOT_MODEL=$CABOT_MODEL"
echo "LIDAR_MODEL=$LIDAR_MODEL"
echo "MAPPING_USE_GNSS=$MAPPING_USE_GNSS"
echo "MAPPING_RESOLUTION=$MAPPING_RESOLUTION"
echo "CONVERT_BAG=$CONVERT_BAG"
echo "gazebo=$gazebo"
echo "points_topic=$points2_topic"
echo "imu_topic=$imu_topic"
echo "fix_topic=$fix_topic"
echo "fix_status_threshold=$fix_status_threshold"

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

# check if CABOT_MODEL is specified
if [ "$robot" = "" ]; then
    echo "CABOT_MODEL must be specified"
    exit
fi
blue "using CABOT_MODEL=$CABOT_MODEL in post processing"

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

# set local variables for file names
if [ $CONVERT_BAG = true ]; then
  bag_file2=${BAG_FILENAME}.carto-converted
else
  bag_file2=${BAG_FILENAME}
fi

samples_file=$bag_file2.loc.samples.json
pbstream_file=$bag_file2.pbstream
log_file=$bag_file2.log
pgm_file=$bag_file2.pgm
yaml_file=$bag_file2.yaml
png_file=$bag_file2.png
info_txt_file=$bag_file2.info.txt
node_options_file=$bag_file2.node-options.yaml
trajectory_csv_file=$bag_file2.trajectory.csv
gnss_csv_file=$bag_file2.gnss.csv

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

if [[ ! -e $WORKDIR/${bag_file2} ]]; then
    ros2 launch mf_localization_mapping convert_rosbag_for_cartographer.launch.py \
	      points2:=${points2_topic} \
	      imu:=${imu_topic} \
	      fix:=${fix_topic} \
	      fix_velocity:=${fix_velocity_topic} \
	      rate:=${PLAYBAG_RATE_PC2_CONVERT} \
	      convert_points:=$convert_points \
	      bag_filename:=$WORKDIR/${BAG_FILENAME}
else
    blue "skipping $WORKDIR/${bag_file2}"
fi

if [[ ! -e $WORKDIR/${samples_file} ]] || [[ ! -e $WORKDIR/${pbstream_file} ]]; then
    cp $configuration_directory_tmp/cartographer_2d_mapping.lua \
          $WORKDIR/${BAG_FILENAME}.cartographer_2d_mapping.lua
    cp $pkg_share_dir/configuration_files/mapping_config.yaml \
          $WORKDIR/${BAG_FILENAME}.mapping_config.yaml

    com="ros2 launch mf_localization_mapping demo_2d_VLP16.launch.py \
	      save_samples:=true \
	      save_state:=true \
	      points2:=${points2_topic} \
	      imu:=${imu_topic} \
	      delay:=10 \
	      rate:=${PLAYBAG_RATE_CARTOGRAPHER} \
	      play_limited_topics:=true \
	      convert_points:=$convert_points \
	      quit_when_rosbag_finish:=${QUIT_WHEN_ROSBAG_FINISH} \
	      fix_status_threshold:=${fix_status_threshold} \
	      grid_resolution:=${MAPPING_RESOLUTION} \
	      configuration_directory:=$configuration_directory_tmp \
	      save_pose:=true \
	      save_trajectory:=true \
	      interpolate_samples_by_trajectory:=true \
	      bag_filename:=$WORKDIR/${bag_file2}"

    if [ $cabot_model != "" ]; then
        com="$com cabot_model:=$cabot_model"
    else
        com="$com robot:=$robot"
    fi

    # outdoor mapping
    if [ "$MAPPING_USE_GNSS" = true ]; then
        cp $configuration_directory/cartographer_2d_mapping_gnss.lua $configuration_directory_tmp
        cp $configuration_directory_tmp/cartographer_2d_mapping_gnss.lua \
              $WORKDIR/${BAG_FILENAME}.cartographer_2d_mapping_gnss.lua

        com="$com \
            fix:=$fix_filtered_topic \
            configuration_basename:=cartographer_2d_mapping_gnss.lua \
            "
    fi

    # copy output to a file
    com="$com | tee $WORKDIR/${log_file}"

    echo $com
    eval $com
else
    blue "skipping $WORKDIR/${samples_file}"
    blue "skipping $WORKDIR/${pbstream_file}"
fi

# convert pbstream to pgm
if [[ ! -e $WORKDIR/${pgm_file} ]]; then
    pushd $WORKDIR
    ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
	   -pbstream_filename ${pbstream_file} \
	   -map_filestem ${bag_file2} \
	   -resolution $MAPPING_RESOLUTION
    popd
else
    blue "skipping $WORKDIR/${pgm_file}"
fi

# convert pgm to png
if [[ ! -e $WORKDIR/${png_file} ]]; then
    convert $WORKDIR/${pgm_file} $WORKDIR/${png_file}
else
    blue "skipping $WORKDIR/${png_file}"
fi

# extract floor map info from the ros map files
if [[ ! -e $WORKDIR/${info_txt_file} ]]; then
    # origin_x, origin_y, ppm
    ros2 run mf_localization_mapping extract_floormap_info_from_yaml.py \
        --input $WORKDIR/${yaml_file} \
        --output $WORKDIR/${info_txt_file}
    # width, height
    identify -format "width: %w\nheight: %h\n" $WORKDIR/${pgm_file} \
        | tee -a $WORKDIR/${info_txt_file}
else
    blue "skipping $WORKDIR/${info_txt_file}"
fi

# post process for mapping with gnss data
if [ "$MAPPING_USE_GNSS" = true ]; then
    # extract option info from log
    if [[ ! -e $WORKDIR/${node_options_file} ]]; then
        ros2 run mf_localization_mapping extract_node_options.py \
            -i $WORKDIR/${log_file} \
            -o $WORKDIR/${node_options_file}
    else
        blue "skipping $WORKDIR/${node_options_file}"
    fi

    # export gnss from bag
    if [[ ! -e $WORKDIR/${gnss_csv_file} ]]; then
        ros2 run mf_localization_mapping export_gnss_fix.py \
            -i $WORKDIR/${bag_file2} \
            -o $WORKDIR/${gnss_csv_file}
    else
        blue "skipping $WORKDIR/${gnss_csv_file}"
    fi

    # get lattiude and longitude used for anchor in error calculation from node options file
    anchor_latitude=$(grep '^options.nav_sat_predefined_enu_frame_latitude' $WORKDIR/${node_options_file} | cut -d ':' -f2)
    anchor_longitude=$(grep '^options.nav_sat_predefined_enu_frame_longitude' $WORKDIR/${node_options_file} | cut -d ':' -f2)

    # calculate error between trajectory and gnss
    ros2 run mf_localization_mapping compare_trajectory_and_gnss.py \
        --trajectory $WORKDIR/${trajectory_csv_file} \
        --gnss $WORKDIR/${gnss_csv_file} \
        --anchor_latitude $anchor_latitude \
        --anchor_longitude $anchor_longitude \
        --plot
fi