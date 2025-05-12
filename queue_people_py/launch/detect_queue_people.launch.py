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
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnShutdown
from cabot_common.launch import AppendLogDirPrefix
from launch.logging import launch_config
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    queue_annotation_list_file = LaunchConfiguration('queue_annotation_list_file')
    debug_without_mf_localization = LaunchConfiguration('debug_without_mf_localization')
    debug_queue_annotation_map_frame = LaunchConfiguration('debug_queue_annotation_map_frame')

    return LaunchDescription([
        # Save all log files in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # Append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("detect_queue_people")])),

        DeclareLaunchArgument(
            'queue_annotation_list_file',
            default_value=f"{get_package_share_directory('queue_people_py')}/annotation/cabot_site_coredo/COREDO-MUJI-202002/coredo_muji_202002_queue_list.yaml",
            description='Path to the queue annotation list file'
        ),
        DeclareLaunchArgument(
            'debug_without_mf_localization',
            default_value='false',
            description='Debug without MF localization'
        ),
        DeclareLaunchArgument(
            'debug_queue_annotation_map_frame',
            default_value='',
            description='Debug queue annotation map frame'
        ),

        Node(
            package='queue_people_py',
            executable='detect_queue_people.py',
            name='detect_queue_people_py',
            output='log',
            parameters=[
                f"{get_package_share_directory('queue_people_py')}/params/queue_detector.yaml",
                {'queue_annotation_list_file': queue_annotation_list_file},
                {'debug_without_mf_localization': debug_without_mf_localization},
                {'debug_queue_annotation_map_frame': debug_queue_annotation_map_frame},
            ]
        ),
    ])
