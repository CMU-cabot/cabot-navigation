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

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    node_name = LaunchConfiguration('node_name')
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    mountpoint = LaunchConfiguration('mountpoint')
    authentificate = LaunchConfiguration('authentificate')
    username = LaunchConfiguration('username')
    password = LaunchConfiguration('password')
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    # options
    relay_back = LaunchConfiguration('relay_back')
    latitude = LaunchConfiguration('latitude')
    longitude = LaunchConfiguration('longitude')
    height = LaunchConfiguration('height')
    nmea_request_cycle = LaunchConfiguration('nmea_request_cycle')

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("str2str")])),

        DeclareLaunchArgument('node_name', default_value='str2str_node'),
        DeclareLaunchArgument('host', default_value=''),
        DeclareLaunchArgument('port', default_value='2101'),
        DeclareLaunchArgument('mountpoint', default_value=''),
        DeclareLaunchArgument('authentificate', default_value='false', description='Whether to authentificate with the server. If set to true, username and password must be supplied.'),
        DeclareLaunchArgument('username', default_value='', description=''),
        DeclareLaunchArgument('password', default_value='', description=''),

        DeclareLaunchArgument('serial_port', default_value='ttyUBLOX'),
        DeclareLaunchArgument('serial_baud', default_value='230400'),
        # options
        DeclareLaunchArgument('relay_back', default_value="0", description='relay back messages from output stream to input stream (-b option) [0]'),
        DeclareLaunchArgument('latitude', default_value="0.0", description='latitude of -p option'),
        DeclareLaunchArgument('longitude', default_value="0.0", description='longitude of -p option'),
        DeclareLaunchArgument('height', default_value="0.0", description='height of -p option'),
        DeclareLaunchArgument('nmea_request_cycle', default_value="0", description='nmea request cycle (ms) [0]'),

        Node(
            package='mf_localization',
            executable='str2str_node.py',
            name='str2str_node',
            parameters=[
                {
                    'node_name': node_name,
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
                }
            ],
        ),
    ])
