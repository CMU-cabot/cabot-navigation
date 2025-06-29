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

from launch import LaunchDescription
from launch.logging import launch_config
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import EnvironmentVariable
from launch.substitutions import PythonExpression
from ament_index_python import get_package_share_directory

from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    output = {'stderr': {'log'}}
    pkg_dir = get_package_share_directory('cabot_ui')
    config_file = LaunchConfiguration('config_file')
    model_name = LaunchConfiguration('model')

    def check_config_file(context):
        import os
        config_file = context.launch_configurations['config_file']
        if not os.path.exists(config_file):
            default_file = PathJoinSubstitution([pkg_dir, 'config', 'cabot_diagnostic.yaml']).perform(context)
            context.launch_configurations['config_file'] = default_file
            return [LogInfo(msg=f"Config file {config_file} not found."),
                    LogInfo(msg=f"Using default config file: {default_file}")]
        return [LogInfo(msg=f"Config file {config_file} found.")]

    return LaunchDescription([
        DeclareLaunchArgument('sigterm_timeout', default_value='15'),
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot_diagnostic")])),
        DeclareLaunchArgument('show_robot_monitor', default_value='true'),
        DeclareLaunchArgument('model', default_value=EnvironmentVariable('CABOT_MODEL'), description='CaBot model'),
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([pkg_dir, 'config', PythonExpression(['"', model_name, '_diagnostic.yaml"'])])),
        OpaqueFunction(function=check_config_file),

        Node(
            package="diagnostic_aggregator",
            executable="aggregator_node",
            name="diagnostic_aggregator",
            parameters=[config_file, {'use_sim_time': True}],
            output=output,
        ),
    ])
