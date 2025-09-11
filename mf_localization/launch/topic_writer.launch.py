#  Copyright (c) 2025  IBM Corporation
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.logging import launch_config
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ParameterValue
from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    output_dir = LaunchConfiguration('output_dir')
    points2 = LaunchConfiguration('points2')
    compressed_image1 = LaunchConfiguration('compressed_image1')
    compressed_image2 = LaunchConfiguration('compressed_image2')
    compressed_image3 = LaunchConfiguration('compressed_image3')
    compressed_image1_throttled = LaunchConfiguration('compressed_image1_throttled')
    compressed_image2_throttled = LaunchConfiguration('compressed_image2_throttled')
    compressed_image3_throttled = LaunchConfiguration('compressed_image3_throttled')
    imu = LaunchConfiguration('imu')
    wifi = LaunchConfiguration('wifi')
    beacons = LaunchConfiguration('beacons')
    fix = LaunchConfiguration('fix')
    verbose = LaunchConfiguration('verbose')
    # output = {'stderr': {'log'}}
    output = "both"

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("mf_localization")])),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value="true",
            description=ParameterValue(True)),

        DeclareLaunchArgument(
            'output_dir',
            default_value=''
            ),

        DeclareLaunchArgument(
            'points2',
            default_value='/velodyne_points'
            ),

        DeclareLaunchArgument(
            'compressed_image1',
            default_value='/rs1/color/image_raw/compressed'
            ),

        DeclareLaunchArgument(
            'compressed_image2',
            default_value='/rs2/color/image_raw/compressed'
            ),

        DeclareLaunchArgument(
            'compressed_image3',
            default_value='/rs3/color/image_raw/compressed'
            ),

        DeclareLaunchArgument(
            'compressed_image1_throttled',
            default_value='/rs1/color/image_throttled/compressed'
            ),

        DeclareLaunchArgument(
            'compressed_image2_throttled',
            default_value='/rs2/color/image_throttled/compressed'
            ),

        DeclareLaunchArgument(
            'compressed_image3_throttled',
            default_value='/rs3/color/image_throttled/compressed'
            ),

        DeclareLaunchArgument(
            'imu',
            default_value='/imu/data'
            ),

        DeclareLaunchArgument(
            'wifi',
            default_value='/esp32/wifi'
            ),

        DeclareLaunchArgument(
            'beacons',
            default_value='/wireless/beacons'
            ),

        DeclareLaunchArgument(
            'fix',
            default_value='/ublox/fix'
            ),

        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='True if output more'),

        # set parameter for all nodes
        SetParameter('use_sim_time', use_sim_time),

        Node(
            package='mf_localization',
            executable='topic_writer_node.py',
            name='topic_writer',
            output=output,
            parameters=[{
                'output_dir': output_dir,
                'verbose': verbose
            }],
            remappings=[
                ('points2', points2),
                ('compressed_image1', compressed_image1),
                ('compressed_image2', compressed_image2),
                ('compressed_image3', compressed_image3),
                ('compressed_image1_throttled', compressed_image1_throttled),
                ('compressed_image2_throttled', compressed_image2_throttled),
                ('compressed_image3_throttled', compressed_image3_throttled),
                ('imu', imu),
                ('wifi', wifi),
                ('beacons', beacons),
                ('fix', fix),
            ]
        ),
    ])
