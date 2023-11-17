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

import os
import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitution import Substitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch import LaunchContext
from typing import List


from launch.logging import launch_config
from cabot_common.launch import AppendLogDirPrefix

class BooleanSubstitution(Substitution):
    """
    Custom substitution class to convert '0/1' or 'false/true' strings to boolean.
    """
    def __init__(self, value: SomeSubstitutionsType):
        self.__value = normalize_to_list_of_substitutions(value)

    def perform(self, context: LaunchContext) -> str:
        value = perform_substitutions(context, self.__value)
        if value in ['1', 'true']:
            return 'true'
        elif value in ['0', 'false']:
            return 'false'
        else:
            raise ValueError(f"Invalid boolean string: {self.__value}")

def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot_ui')
    output = 'both'

    show_gazebo = LaunchConfiguration('show_gazebo')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_config_file2 = LaunchConfiguration('rviz_config_file2')
    show_rviz = LaunchConfiguration('show_rviz')
    show_local_rviz = LaunchConfiguration('show_local_rviz')
    show_robot_monitor = LaunchConfiguration('show_robot_monitor')
    use_sim_time = LaunchConfiguration('use_sim_time')

    show_gazebo_ = BooleanSubstitution(show_gazebo)
    show_rviz_ = BooleanSubstitution(show_rviz)
    show_local_rviz_ = BooleanSubstitution(show_local_rviz)
    show_robot_monitor_ = BooleanSubstitution(show_robot_monitor)
    use_sim_time_ = BooleanSubstitution(use_sim_time)

    check_gazebo_ready = Node(
        package='cabot_gazebo',
        executable='check_gazebo_ready.py',
        name='check_gazebo_ready_node',
        parameters=[{'check_mobile_base': True}],
        condition=IfCondition(use_sim_time_),
    )

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot_gui")])),

        DeclareLaunchArgument(
            'show_gazebo',
            default_value=EnvironmentVariable('CABOT_SHOW_GAZEBO_CLIENT', default_value='false'),
            description='Show Gazebo client if true'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(pkg_dir, 'rviz', 'nav2_default_view.rviz'),
            description='Full path to the RVIZ config file to use'),

        DeclareLaunchArgument(
            'rviz_config_file2',
            default_value=os.path.join(pkg_dir, 'rviz', 'nav2_default_view_local.rviz'),
            description='Full path to the RVIZ config file to use'),

        DeclareLaunchArgument(
            'show_rviz',
            default_value=EnvironmentVariable('CABOT_SHOW_ROS2_RVIZ', default_value='true'),
            description='Show rviz2 if true'
        ),
        DeclareLaunchArgument(
            'show_local_rviz',
            default_value=EnvironmentVariable('CABOT_SHOW_ROS2_LOCAL_RVIZ', default_value='false'),
            description='Show rviz2 for local if true'
        ),
        DeclareLaunchArgument(
            'show_robot_monitor',
            default_value=EnvironmentVariable('CABOT_SHOW_ROBOT_MONITOR', default_value='true'),
            description='Show robot monitor if true'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=EnvironmentVariable('CABOT_USE_SIM_TIME', default_value='true'),
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'),
                                            '/launch/gzclient.launch.py']),
            condition=IfCondition(AndSubstitution(left=use_sim_time_, right=show_gazebo_)),
            launch_arguments={
                'verbose': 'true'
            }.items()
        ),
        LogInfo(msg='non gazebo env'),    
        Node(
            condition=IfCondition(show_rviz_),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time_}],
            output='log',
        ),
        Node(
            condition=IfCondition(show_local_rviz_),
            package='rviz2',
            executable='rviz2',
            name='rviz2_local',
            namespace='local',
            arguments=['-d', rviz_config_file2],
            parameters=[{'use_sim_time': use_sim_time_}],
            output='log',
        ),
        Node(
            package="rqt_robot_monitor",
            executable="rqt_robot_monitor",
            name="rqt_robot_monitor",
            condition=IfCondition(show_robot_monitor_)
        ),
    ])
