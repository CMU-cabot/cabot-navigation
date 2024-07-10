#!/usr/bin/env python3
# Copyright (c) 2014  IBM Corporation and Carnegie Mellon University
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
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    mountpoint = LaunchConfiguration('mountpoint')
    authentificate = LaunchConfiguration('authentificate')
    username = LaunchConfiguration('username')
    password = LaunchConfiguration('password')
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("str2str")])),

        DeclareLaunchArgument('host', default_value='', description=''),
        DeclareLaunchArgument('port', default_value='2101', description=''),
        DeclareLaunchArgument('mountpoint', default_value='', description=''),
        DeclareLaunchArgument('authentificate', default_value='false', description=''),
        DeclareLaunchArgument('username', default_value='', description=''),
        DeclareLaunchArgument('password', default_value='', description=''),
        DeclareLaunchArgument('serial_port', default_value='ttyUBLOX', description=''),
        DeclareLaunchArgument('serial_baud', default_value='230400', description=''),

        Node(
            package='mf_localization',
            executable='str2str_node.py',
            name='str2str_node',
            parameters=[
                {
                    "host": host,
                    "port": port,
                    "mountpoint": mountpoint,
                    "authentificate": authentificate,
                    "username": username,
                    "password": password,
                    "serial_port": serial_port,
                    "serial_baud": serial_baud,
                }
            ],
        ),
    ])
