#!/usr/bin/env python3
# Copyright (c) 2021, 2023  IBM Corporation and Carnegie Mellon University

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import datetime

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetLaunchConfiguration
from launch.actions import LogInfo
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationNotEquals
from launch.conditions import UnlessCondition
from launch.substitutions import AndSubstitution
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ParameterFile
from launch.utilities import normalize_to_list_of_substitutions


def generate_launch_description():
    pkg_dir = get_package_share_directory('mf_localization_mapping')

    param_files = [
        ParameterFile(PathJoinSubstitution([
            pkg_dir,
            'configuration_files',
            'mapping_config.yaml'
        ]),
            allow_substs=True,
        )
    ]

    robot = LaunchConfiguration('robot')
    cabot_model = LaunchConfiguration('cabot_model')
    run_cartographer = LaunchConfiguration('run_cartographer')
    record_bag = LaunchConfiguration('record_bag')
    prefix = LaunchConfiguration('prefix')
    scan = LaunchConfiguration('scan')
    save_samples = LaunchConfiguration('save_samples')
    save_trajectory = LaunchConfiguration('save_trajectory')
    save_pose = LaunchConfiguration('save_pose')
    wireless_topics = LaunchConfiguration('wireless_topics')
    bag_filename = LaunchConfiguration('bag_filename')
    load_state_filename = LaunchConfiguration('load_state_filename')
    save_state_filename = LaunchConfiguration('save_state_filename')
    record_required = LaunchConfiguration('record_required')
    record_points = LaunchConfiguration('record_points')
    compression_mode = LaunchConfiguration('compression_mode')
    configuration_basename = LaunchConfiguration('configuration_basename')
    # save_state_filename # todo
    start_trajectory_with_default_topics = LaunchConfiguration('start_trajectory_with_default_topics')
    record_wireless = LaunchConfiguration('record_wireless')

    use_xsens = LaunchConfiguration('use_xsens')
    use_arduino = LaunchConfiguration('use_arduino')
    use_esp32 = LaunchConfiguration('use_esp32')
    use_velodyne = LaunchConfiguration('use_velodyne')
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic = LaunchConfiguration('imu_topic')
    fix_topic = LaunchConfiguration('fix_topic')

    save_empty_beacon_sample = LaunchConfiguration('save_empty_beacon_sample')

    # launch configurations updated in the launch description
    bag_filename_fullpath = LaunchConfiguration('bag_filename_fullpath')
    saved_location = LaunchConfiguration('saved_location')
    save_state_directory = LaunchConfiguration('save_state_directory')
    save_state_filestem = LaunchConfiguration('save_state_filestem')

    fix_status_threshold = LaunchConfiguration('fix_status_threshold')
    fix_overwrite_time = LaunchConfiguration('fix_overwrite_time')
    interpolate_samples_by_trajectory = LaunchConfiguration('interpolate_samples_by_trajectory')

    def configure_ros2_bag_arguments(context, node):
        cmd = node.cmd.copy()
        if use_sim_time.perform(context) == 'true':
            cmd.extend(['--use-sim-time'])
        if compression_mode.perform(context) != 'none':
            cmd.extend(['--compression-mode', compression_mode, '--compression-format', 'zstd'])
        if record_required.perform(context) == 'true':
            if record_points.perform(context) == 'true':
                cmd.extend(['-a', '-x', "'/map|(.*)points_cropped|/pandar_packets|(.*)/image_raw|(.*)/image_raw/(.*)'"])
            else:
                cmd.extend(['-a', '-x', "'/map|/velodyne_points|(.*)points_cropped|/pandar_packets|(.*)/image_raw|(.*)/image_raw/(.*)'"])
        else:
            cmd.append('-a')
        saved_location_temp = []
        save_state_directory_temp = [EnvironmentVariable('HOME'), '/recordings/']
        save_state_filestem_temp = []
        if bag_filename.perform(context) == '':
            now_string = datetime.datetime.now().strftime("_%Y_%m_%d-%H_%M_%S")
            save_state_filestem_temp = [prefix, now_string]
            saved_location_temp = save_state_directory_temp + save_state_filestem_temp
            cmd.extend(['-o', saved_location_temp])
        else:
            save_state_filestem_temp = [bag_filename]
            saved_location_temp = save_state_directory_temp + save_state_filestem_temp
            cmd.extend(['-o', saved_location_temp])
        node.cmd.clear()
        node.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd])
        return [node,
                SetLaunchConfiguration('bag_filename_fullpath', saved_location_temp),
                SetLaunchConfiguration('saved_location', ['./docker/home']+saved_location_temp[1:]),
                SetLaunchConfiguration('save_state_directory', save_state_directory_temp),
                SetLaunchConfiguration('save_state_filestem', save_state_filestem_temp),
                ]

    ros2_bag_process = ExecuteProcess(
        cmd=["ros2", "bag", "record"]
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='rover'),
        DeclareLaunchArgument('cabot_model', default_value=''),
        DeclareLaunchArgument('run_cartographer', default_value='true'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('prefix', default_value='sensor'),
        DeclareLaunchArgument('scan', default_value='velodyne_scan'),
        DeclareLaunchArgument("save_samples", default_value="false"),
        DeclareLaunchArgument('save_trajectory', default_value='false'),
        DeclareLaunchArgument('save_pose', default_value='false'),
        DeclareLaunchArgument("wireless_topics", default_value="['/wireless/beacons','/wireless/wifi','/esp32/wifi']"),

        DeclareLaunchArgument("bag_filename", default_value=""),
        DeclareLaunchArgument("load_state_filename", default_value=""),
        DeclareLaunchArgument("record_required", default_value="false"),
        DeclareLaunchArgument("record_points", default_value="false"),
        DeclareLaunchArgument("compression_mode", default_value="message", description="{none,file,message} compress bag"),

        DeclareLaunchArgument("configuration_basename", default_value="cartographer_2d_mapping.lua"),
        DeclareLaunchArgument("save_state_filename", default_value=""),
        DeclareLaunchArgument("start_trajectory_with_default_topics", default_value="true"),
        SetLaunchConfiguration(name="start_trajectory_with_default_topics", value="false", condition=LaunchConfigurationNotEquals("load_state_filename", "")),

        DeclareLaunchArgument("record_wireless", default_value="false"),

        DeclareLaunchArgument("use_xsens", default_value="true"),
        DeclareLaunchArgument("use_arduino", default_value="false"),
        DeclareLaunchArgument("use_esp32", default_value="false"),
        DeclareLaunchArgument("use_velodyne", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("imu_topic", default_value="imu/data"),
        DeclareLaunchArgument('fix_topic', default_value='ublox/fix'),

        DeclareLaunchArgument('save_empty_beacon_sample', default_value='true'),

        DeclareLaunchArgument('fix_status_threshold', default_value='2'),
        DeclareLaunchArgument('fix_overwrite_time', default_value='false'),
        DeclareLaunchArgument('interpolate_samples_by_trajectory', default_value='false'),

        SetParameter('use_sim_time', use_sim_time),

        LogInfo(
            msg="use_arduino and use_xsens are both true",
            condition=IfCondition(AndSubstitution(use_xsens, use_arduino)),
        ),

        GroupAction([
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'includes', 'xsens_driver_cartographer.launch.py'])),
                condition=IfCondition(use_xsens)
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'includes', 'arduino_cartographer.launch.xml'])),
                condition=IfCondition(use_arduino)
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'includes', 'esp32_cartographer.launch.xml'])),
                condition=IfCondition(use_esp32)
            ),

            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'includes', 'VLP16_points_cartographer.launch.py'])),
                launch_arguments={
                    "scan": scan
                }.items(),
                condition=IfCondition(use_velodyne)
            ),
            # launch esp32 wifi receiver when esp32 is used as a primary micro controller
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory('wireless_scanner_ros'), 'launch', 'esp32_receiver.launch.xml'])),
                condition=IfCondition(AndSubstitution(record_wireless, use_esp32))
            ),
            # ble_receiver will be launched by ble_scan service
            # IncludeLaunchDescription(
            #     AnyLaunchDescriptionSource(PathJoinSubstitution([get_package_share_directory('wireless_scanner_ros'), 'launch', 'ble_receiver.launch.xml'])),
            #     condition=IfCondition(record_wireless)
            # ),

            OpaqueFunction(
                function=configure_ros2_bag_arguments,
                args=[ros2_bag_process],
                condition=IfCondition(record_bag)
            ),
            # set save_state_filename after configure_ros2_bag_arguments
            SetLaunchConfiguration(
                name="save_state_filename",
                value=[bag_filename_fullpath, ".pbstream"],
                condition=LaunchConfigurationNotEquals("bag_filename", "")
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_dir, 'launch', 'cartographer_2d_VLP16.launch.py'])),
                launch_arguments={
                    "robot": robot,
                    "cabot_model": cabot_model,
                    "scan": scan,
                    "configuration_basename": configuration_basename,
                    "load_state_filename": load_state_filename,
                    "save_state_filename": save_state_filename,
                    "start_trajectory_with_default_topics": start_trajectory_with_default_topics,
                    "imu": imu_topic,
                    "fix": fix_topic,
                    "convert_pbstream": "true",
                    "save_state_directory": save_state_directory,
                    "save_stete_filestem": save_state_filestem,
                }.items(),
                condition=IfCondition(run_cartographer)
            ),

            Node(
                package='mf_localization',
                executable='fix_filter.py',
                name='ublox_fix_filter',
                parameters=[
                    *param_files,
                    {
                        'status_threshold': fix_status_threshold,
                        'overwrite_time': fix_overwrite_time,
                    }
                ],
                remappings=[
                    ('fix', 'ublox/fix'),
                    ('fix_filtered', 'ublox/fix_filtered')
                ]
            ),

            Node(
                package='mf_localization',
                executable='ublox_converter.py',
                name='ublox_converter',
                parameters=[
                    *param_files
                ],
                remappings=[
                    ('navsat', 'ublox/navsat')
                ]
            ),

            GroupAction([
                SetParameter(name="output", value=[bag_filename_fullpath, ".loc.samples.json"], condition=LaunchConfigurationNotEquals('bag_filename', '')),
                Node(
                    package="mf_localization",
                    executable="tf2_beacons_listener.py",
                    name="tf2_beacons_listener",
                    parameters=[{
                        "topics": wireless_topics,
                        'save_empty_beacon_sample': save_empty_beacon_sample,
                        'output_trajectory': PythonExpression(['"', bag_filename_fullpath, '.trajectory.csv" if "', save_trajectory, '"=="true" else ""']),
                        'trajectory_recorder_timer_period': 10.0,
                        'interpolate_by_trajectory': interpolate_samples_by_trajectory,
                    }],
                    condition=IfCondition(save_samples)
                ),
            ]),

            # write pose to csv file
            Node(
                name="tracked_pose_listener",
                package="mf_localization",
                executable="tracked_pose_listener.py",
                parameters=[
                    *param_files,
                    {
                        "output": PythonExpression(['"', bag_filename_fullpath, '.tracked_pose.csv" if "', save_pose, '"=="true" else ""'])
                    }
                ],
                condition=IfCondition(save_pose),
            ),

            Node(
                name="rviz2",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg_dir, 'configuration_files', 'rviz', 'demo_2d.rviz'])],
                condition=IfCondition(run_cartographer)
            ),

            RegisterEventHandler(
                OnProcessExit(
                    target_action=ros2_bag_process,
                    on_exit=[
                        LogInfo(
                            msg=['Mapping data is saved at ', saved_location]
                        )
                    ]
                )
            ),
            ],
            scoped=False,  # disable scoping to allow registered events to have access to locally update launch configurations
            condition=UnlessCondition(AndSubstitution(use_xsens, use_arduino))
        )
    ])
