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

"""
Launch file for remote control
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot_ui')

    gamepad = LaunchConfiguration('gamepad')
    cmd_vel = LaunchConfiguration('cmd_vel')

    param_files = [
        ParameterFile(PathJoinSubstitution([
                pkg_dir,
                'config',
                'cabot2-common-mapping.yaml'
            ]), allow_substs=True
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'gamepad',
            default_value='ps4',
            description='Gamepad name ps4 or pro (Switch Pro Con)'
        ),

        DeclareLaunchArgument(
            'cmd_vel',
            default_value='/cmd_vel',
            description='Command velocity topic published by teleop_twist_joy node'
        ),

        Node(
            package='joy',
            executable='joy_node',
            namespace='cabot',
            name='joy_node',
            parameters=[*param_files],
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            namespace='cabot',
            name=PythonExpression(['"teleop_twist_joy_', gamepad, '"']),
            parameters=[*param_files],
            remappings=[
                ("/cabot/cmd_vel", cmd_vel)
            ]
        ),
    ])
