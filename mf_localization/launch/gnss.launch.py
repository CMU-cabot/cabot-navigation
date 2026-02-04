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
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution
from launch.substitutions import NotSubstitution
from launch_ros.actions import Node
from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():

    mf_localization_dir = get_package_share_directory('mf_localization')
    ntrip_client_dir = get_package_share_directory('ntrip_client')

    str2str_node = LaunchConfiguration('str2str_node')
    ntrip_client = LaunchConfiguration('ntrip_client')
    ublox_node = LaunchConfiguration('ublox_node')
    str2str_node_logger = LaunchConfiguration('str2str_node_logger')
    ntrip_client_logger = LaunchConfiguration('ntrip_client_logger')
    ublox_node_logger = LaunchConfiguration('ublox_node_logger')

    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    mountpoint = LaunchConfiguration('mountpoint')
    authentificate = LaunchConfiguration('authentificate')
    username = LaunchConfiguration('username')
    password = LaunchConfiguration('password')
    respawn_ntrip_client = LaunchConfiguration('respawn_ntrip_client')

    # str2str_node
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    # options
    relay_back = LaunchConfiguration('relay_back')
    latitude = LaunchConfiguration('latitude')
    longitude = LaunchConfiguration('longitude')
    height = LaunchConfiguration('height')
    nmea_request_cycle = LaunchConfiguration('nmea_request_cycle')

    # cabot use gnss
    reconnect_error_level = LaunchConfiguration('reconnect_error_level')
    rtcm_timeout_error_level = LaunchConfiguration('rtcm_timeout_error_level')
    fix_warn_error_level = LaunchConfiguration('fix_warn_error_level')
    no_fix_error_level = LaunchConfiguration('no_fix_error_level')

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
                                'reconnect_attempt_max': '10000',  # default: 10
                                'respawn': respawn_ntrip_client,
                                'nmea_max_length': '90',  # a large value to accept high precision mode
                                'reconnect_error_level': reconnect_error_level,
                                'rtcm_timeout_error_level': rtcm_timeout_error_level,
                            }.items(),
                            condition=IfCondition(ntrip_client)
    )

    ublox_node_launch = IncludeLaunchDescription(
                            FrontendLaunchDescriptionSource([mf_localization_dir + "/launch/ublox-zed-f9p.launch.xml"]),
                            launch_arguments={
                                'node_name': 'ublox',
                                'fix_warn_error_level': fix_warn_error_level,
                                'no_fix_error_level': no_fix_error_level,
                            }.items(),
                            condition=IfCondition(AndSubstitution(ublox_node, NotSubstitution(str2str_node)))  # if ublox_node and (not str2str_node)
                        )

    # ublox_node must be launched after str2str_node to prevent simultaneous access to the device
    wait_and_ublox_node_launch = TimerAction(
                                    period=5.0,
                                    actions=[
                                        LogInfo(msg="launch ublox_node 5 seconds after launching str2str_node"),
                                        IncludeLaunchDescription(
                                            FrontendLaunchDescriptionSource([mf_localization_dir + "/launch/ublox-zed-f9p.launch.xml"]),
                                            launch_arguments={
                                                'node_name': 'ublox',
                                                'fix_warn_error_level': fix_warn_error_level,
                                                'no_fix_error_level': no_fix_error_level,
                                            }.items()
                                        )
                                    ],
                                    condition=IfCondition(AndSubstitution(ublox_node, str2str_node))  # if ublox_node and str2str_node
                                )

    str2str_node_logger_launch = Node(
            package='cabot_common',
            executable='log_redirector_node',
            name='str2str_node_log_redirector',
            parameters=[
                {'target_node': 'str2str_node'}
            ],
            output={},
            condition=IfCondition(str2str_node_logger)
        )

    ntrip_client_logger_launch = Node(
            package='cabot_common',
            executable='log_redirector_node',
            name='ntrip_client_log_redirector',
            parameters=[
                {'target_node': 'ntrip_client'}
            ],
            output={},
            condition=IfCondition(ntrip_client_logger)
        )

    ublox_node_logger_launch = Node(
            package='cabot_common',
            executable='log_redirector_node',
            name='ublox_node_log_redirector',
            parameters=[
                {'target_node': 'ublox'}
            ],
            output={},
            condition=IfCondition(ublox_node_logger)
        )

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("gnss")])),

        DeclareLaunchArgument('str2str_node', default_value='false', description='Whether to launch str2str node [false]'),
        DeclareLaunchArgument('ntrip_client', default_value='false', description='Whether to launch ntrip_client node [false]'),
        DeclareLaunchArgument('ublox_node', default_value='false', description='Whether to launch ublox_node [false]'),
        DeclareLaunchArgument('str2str_node_logger', default_value='false', description='Whether to launch str2str_node_logger [false]'),
        DeclareLaunchArgument('ntrip_client_logger', default_value='false', description='Whether to launch ntrip_client_logger [false]'),
        DeclareLaunchArgument('ublox_node_logger', default_value='false', description='Whether to launch ublox_node_logger [false]'),

        # ntrip_client and str2str_node-
        DeclareLaunchArgument('host', default_value=''),
        DeclareLaunchArgument('port', default_value='2101'),
        DeclareLaunchArgument('mountpoint', default_value=''),
        DeclareLaunchArgument('authentificate', default_value='false', description='Whether to authentificate with the server. If set to true, username and password must be supplied.'),
        DeclareLaunchArgument('username', default_value=''),
        DeclareLaunchArgument('password', default_value=''),
        DeclareLaunchArgument('respawn_ntrip_client', default_value='true'),

        # str2str_node
        DeclareLaunchArgument('serial_port', default_value='ttyUBLOX'),
        DeclareLaunchArgument('serial_baud', default_value='230400'),
        # str2str_node options
        DeclareLaunchArgument('relay_back', default_value="0", description='relay back messages from output stream to input stream (-b option) [0]'),
        DeclareLaunchArgument('latitude', default_value="0.0", description='latitude of -p option'),
        DeclareLaunchArgument('longitude', default_value="0.0", description='longitude of -p option'),
        DeclareLaunchArgument('height', default_value="0.0", description='height of -p option'),
        DeclareLaunchArgument('nmea_request_cycle', default_value="0", description='nmea request cycke (ms) [0]'),

        # cabot use gnss
        # ntrip client
        DeclareLaunchArgument('reconnect_error_level', default_value='1', description='Error level for NTRIP client reconnection. default: 1 (WARN)'),
        DeclareLaunchArgument('rtcm_timeout_error_level', default_value='1', description='Error level for RTCM timeout. default: 1 (WARN)'),
        # ublox node
        DeclareLaunchArgument('fix_warn_error_level', default_value='1', description='Error level for inaccurate FIX status. default: 1 (WARN)'),
        DeclareLaunchArgument('no_fix_error_level', default_value='1', description='Error level for NO FIX status. default: 1 (WARN)'),

        # node actions
        str2str_node_launch,
        ntrip_client_launch,
        ublox_node_launch,
        wait_and_ublox_node_launch,

        # logger node actions
        str2str_node_logger_launch,
        ntrip_client_logger_launch,
        ublox_node_logger_launch,
    ])
