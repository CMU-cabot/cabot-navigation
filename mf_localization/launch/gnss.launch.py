#!/usr/bin/env python3
# Copyright (c) 2024  IBM Corporation
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():

    mf_localization_dir = get_package_share_directory('mf_localization')
    ntrip_client_dir = get_package_share_directory('ntrip_client')

    str2str_node = LaunchConfiguration('str2str_node')
    ntrip_client = LaunchConfiguration('ntrip_client')
    ublox_node = LaunchConfiguration('ublox_node')

    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    mountpoint = LaunchConfiguration('mountpoint')
    authentificate = LaunchConfiguration('authentificate')
    username = LaunchConfiguration('username')
    password = LaunchConfiguration('password')

    # str2str_node
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    # options
    relay_back = LaunchConfiguration('relay_back')
    latitude = LaunchConfiguration('latitude')
    longitude = LaunchConfiguration('longitude')
    height = LaunchConfiguration('height')
    nmea_request_cycle = LaunchConfiguration('nmea_request_cycle')

    str2str_node_launch = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource([mf_localization_dir + "/launch/str2str.launch.py"]),
                            launch_arguments={
                                'node_name': 'str2str_node',
                                'host': host,
                                'port': port,
                                'mountpoint': mountpoint,
                                'authentificate': authentificate,
                                'username': username,
                                'password': password,
                                'serial_port': serial_port,
                                'serial_baud': serial_baud,
                                'relay_back': relay_back,
                                'latitude': latitude,
                                'longitude': longitude,
                                'height': height,
                                'nmea_request_cycle': nmea_request_cycle
                            }.items(),
                            condition=IfCondition(str2str_node)
                        )

    ntrip_client_launch = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource([ntrip_client_dir + "/ntrip_client_launch.py"]),
                            launch_arguments={
                                'node_name': 'ntrip_client',
                                'host': host,
                                'port': port,
                                'mountpoint': mountpoint,
                                'authentificate': authentificate,
                                'username': username,
                                'password': password,
                                'nmea_max_length': '90',  # a large value to accept high precision mode
                            }.items(),
                            condition=IfCondition(ntrip_client)
    )

    ublox_node_launch = IncludeLaunchDescription(
                            FrontendLaunchDescriptionSource([mf_localization_dir + "/launch/ublox-zed-f9p.launch.xml"]),
                            launch_arguments={
                                'node_name': 'ublox',
                            }.items(),
                            condition=IfCondition(ublox_node)
                        )

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("gnss")])),

        DeclareLaunchArgument('str2str_node', default_value='false', description=''),
        DeclareLaunchArgument('ntrip_client', default_value='false', description=''),
        DeclareLaunchArgument('ublox_node', default_value='false', description=''),

        DeclareLaunchArgument('host', default_value='', description=''),
        DeclareLaunchArgument('port', default_value='2101', description=''),
        DeclareLaunchArgument('mountpoint', default_value='', description=''),
        DeclareLaunchArgument('authentificate', default_value='false', description=''),
        DeclareLaunchArgument('username', default_value='', description=''),
        DeclareLaunchArgument('password', default_value='', description=''),

        # str2str_node
        DeclareLaunchArgument('serial_port', default_value='ttyUBLOX', description=''),
        DeclareLaunchArgument('serial_baud', default_value='230400', description=''),
        # options
        DeclareLaunchArgument('relay_back', default_value="0", description=''),
        DeclareLaunchArgument('latitude', default_value="0.0", description=''),
        DeclareLaunchArgument('longitude', default_value="0.0", description=''),
        DeclareLaunchArgument('height', default_value="0.0", description=''),
        DeclareLaunchArgument('nmea_request_cycle', default_value="0", description='nmea request cycke (ms) [0]'),

        str2str_node_launch,
        ntrip_client_launch,
        ublox_node_launch
    ])
