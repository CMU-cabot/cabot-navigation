#!/usr/bin/env python3
# Copyright (c) 2021, 2022  IBM Corporation and Carnegie Mellon University
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

from launch.logging import launch_config

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.actions import SetParametersFromFile
from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    output = {'stderr': {'log'}}
    pkg_dir = get_package_share_directory('mf_localization')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot = LaunchConfiguration('robot')
    rssi_offset = LaunchConfiguration('rssi_offset')

    map_config_file = LaunchConfiguration('map_config_file')
    tags = LaunchConfiguration('tags')
    beacons_topic = LaunchConfiguration('beacons_topic')
    wifi_topic = LaunchConfiguration('wifi_topic')
    with_odom_topic = LaunchConfiguration('with_odom_topic')

    points2_topic = LaunchConfiguration('points2_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    pressure_available = LaunchConfiguration('pressure_available')
    pressure_topic = LaunchConfiguration('pressure_topic')

    use_gnss = LaunchConfiguration('use_gnss')
    use_global_localizer = LaunchConfiguration('use_global_localizer')
    gnss_fix_topic = LaunchConfiguration('gnss_fix_topic')
    gnss_fix_velocity_topic = LaunchConfiguration('gnss_fix_velocity_topic')
    gnss_navsat_topic = LaunchConfiguration('gnss_navsat_topic')

    publish_current_rate = LaunchConfiguration('publish_current_rate')

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("mf_localization")])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        DeclareLaunchArgument('robot', default_value='', description='Robot type should be specified'),
        DeclareLaunchArgument('rssi_offset', default_value='0.0', description='Set RSSI offset to estimate location'),

        DeclareLaunchArgument('map_config_file', default_value='', description='Specify map config file path'),
        DeclareLaunchArgument('tags', default_value='', description='Specify tags to select which maps to load (comma-separated string e.g. v0.2,demo)'),
        DeclareLaunchArgument('beacons_topic', default_value='beacons', description='Specify beacons topic name'),
        DeclareLaunchArgument('wifi_topic', default_value='wifi', description='Specify wifi topic name'),
        DeclareLaunchArgument('with_odom_topic', default_value='false', description='Weather odom topic is used to cartographer localization or not'),

        DeclareLaunchArgument('points2_topic', default_value='velodyne_points', description='Specify velodyne points topic name'),
        DeclareLaunchArgument('imu_topic', default_value='imu/data', description='Specify IMU topic name'),
        DeclareLaunchArgument('odom_topic', default_value='odom', description='Specify odometry topic name'),
        DeclareLaunchArgument('pressure_available', default_value='true', description='Weather pressure topic is available or not'),
        DeclareLaunchArgument('pressure_topic', default_value='pressure', description='Specify pressure topic name'),

        DeclareLaunchArgument('use_gnss', default_value='false', description='Weather GNSS functionality is used or not'),
        DeclareLaunchArgument('use_global_localizer', default_value='false', description='Weather use global localizer or not'),
        DeclareLaunchArgument('gnss_fix_topic', default_value='gnss_fix', description='Specify GNSS fix topic name'),
        DeclareLaunchArgument('gnss_fix_velocity_topic', default_value='gnss_fix_velocity', description='Specify GNSS fix velocity topicname'),
        DeclareLaunchArgument('gnss_navsat_topic', default_value='ublox/navsat', description='Specify NAVSAT topicname'),

        DeclareLaunchArgument('publish_current_rate', default_value='0', description='Specify the rate of publishing current location'),

        GroupAction([
            SetParametersFromFile(
                PathJoinSubstitution([pkg_dir, 'configuration_files', 'multi_floor', 'multi_floor_manager_with_odom.yaml']),
                condition=IfCondition(with_odom_topic),
            ),
            SetParametersFromFile(
                PathJoinSubstitution([pkg_dir, 'configuration_files', 'multi_floor', 'multi_floor_manager.yaml']),
                condition=UnlessCondition(with_odom_topic),
            ),
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='mf_localization',
                executable='multi_floor_topic_proxy',
                name='multi_floor_topic_proxy',
                output=output,
                parameters=[{
                    'map_config_file': map_config_file,
                    'verbose': True,
                }],
                remappings=[
                    ('points2', points2_topic),
                    ('imu', imu_topic),
                    ('odom', odom_topic),
                ],
                condition=IfCondition("true"),
            ),
            Node(
                package='mf_localization',
                executable='multi_floor_manager.py',
                name='multi_floor_manager',
                output=output,
                parameters=[{
                    'map_config_file': map_config_file,
                    'tags': tags,
                    'configuration_directory': pkg_dir+'/configuration_files/cartographer',
                    'configuration_file_prefix': 'cartographer_2d',
                    'robot': robot,
                    'rssi_offset': rssi_offset,
                    'use_gnss': use_gnss,
                    'use_global_localizer': use_global_localizer,
                    'publish_current_rate': publish_current_rate,
                    'pressure_available': pressure_available,
                    'verbose': True,
                }],
                remappings=[
                    ('beacons', beacons_topic),
                    ('wifi', wifi_topic),
                    ('points2', points2_topic),
                    ('imu', imu_topic),
                    ('odom', odom_topic),
                    ('pressure', pressure_topic),
                    ('gnss_fix', gnss_fix_topic),
                    ('gnss_fix_velocity', gnss_fix_velocity_topic),
                    # ublox_converter
                    ('navsat', gnss_navsat_topic),
                    ('num_active_sv', 'ublox_converter/num_active_sv'),
                    ('sv_status', 'ublox_converter/sv_status'),
                ],
                # prefix='python3 -m cProfile -o multi_floor_manager.profile',
            ),
        ]),
    ])
