# Copyright (c) 2025  Carnegie Mellon University
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.event_handlers import OnShutdown
from cabot_common.launch import AppendLogDirPrefix
from launch.logging import launch_config


def generate_launch_description():
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument('sigterm_timeout', default_value='30'),
        # Save all log files in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # Append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot_ext_ble")])),

        DeclareLaunchArgument(
            'port',
            default_value=EnvironmentVariable('CABOT_ROSBRIDGE_PORT', default_value='9091'),
            description='Port for rosbridge server'
        ),
        # rosbridge for external BLE server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_server',
            output={},
            parameters=[{'port': port, 'max_message_size': 128000000}]
        ),
        # Node for map viewing on smartphone device (roslibjs/ros3djs)
        Node(
            package='tf2_web_republisher',
            executable='tf2_web_republisher',
            name='tf2_web_republisher',
            output={}
        ),
    ])
