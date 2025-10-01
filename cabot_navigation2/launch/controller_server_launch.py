# Copyright (c) 2020  Carnegie Mellon University
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

from launch.logging import launch_config
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import EnvironmentVariable


from nav2_common.launch import RewrittenYaml
from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('cabot_navigation2')
    output = {'stderr': {'log'}}

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    params_file2 = LaunchConfiguration('params_file2')
    default_bt_xml_file = LaunchConfiguration('default_bt_xml_file')
    default_bt_xml_file2 = LaunchConfiguration('default_bt_xml_file2')
    autostart = LaunchConfiguration('autostart')
    footprint_radius = LaunchConfiguration('footprint_radius')
    offset = LaunchConfiguration('offset')
    cabot_side = LaunchConfiguration('cabot_side')
    low_obstacle_detect_version = LaunchConfiguration('low_obstacle_detect_version')
    publish_low_obstacle_ground = LaunchConfiguration('publish_low_obstacle_ground')
    footprint_publisher_version = LaunchConfiguration('footprint_publisher_version')

    use_low_obstacle_detect = PythonExpression([low_obstacle_detect_version, " > 0"])

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    remappings2 = [('/local/tf', 'local/tf'),
                   ('/local/tf_static', 'local/tf_static'),
                   ('/local/cmd_vel', '/cmd_vel'),
                   ('/local/odom', '/odom'),
                   ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'default_bt_xml_filename': default_bt_xml_file,
        # 'footprint_normal': footprint_radius,
        'robot_radius': footprint_radius,
        'inflation_radius': PythonExpression([footprint_radius, "+ 0.30"]),
        'offset_sign': PythonExpression(["-1.0 if '", cabot_side, "'=='right' else +1.0"]),
        'offset_normal': offset
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    param_substitutions2 = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'default_bt_xml_filename': default_bt_xml_file2,
        # 'footprint_normal': footprint_radius,
        'robot_radius': footprint_radius,
        'offset_sign': PythonExpression(["-1.0 if '", cabot_side, "'=='right' else +1.0"]),
        'offset_normal': offset
    }

    configured_params2 = RewrittenYaml(
        source_file=params_file2,
        root_key="local",
        param_rewrites=param_substitutions2,
        convert_types=True)

    return LaunchDescription([
        DeclareLaunchArgument('sigterm_timeout', default_value='15'),
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot_navigation2")])),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'params_file2',
            default_value=os.path.join(pkg_dir, 'params', 'nav2_params2.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'default_bt_xml_file',
            default_value=os.path.join(
                get_package_share_directory('cabot_bt'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_file2',
            default_value=os.path.join(
                get_package_share_directory('cabot_bt'),
                'behavior_trees', 'navigate_w_local_odom.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'use_remappings', default_value='true',
            description='Arguments to pass to all nodes launched by the file'),

        DeclareLaunchArgument(
            'footprint_radius', default_value='0.45',
            description='Normal footprint radius'),

        DeclareLaunchArgument(
            'offset', default_value='0.25',
            description='Normal offset'),

        DeclareLaunchArgument(
            'cabot_side', default_value='left',
            description='cabot side (left -> user stands right) left/right'),

        DeclareLaunchArgument(
            'low_obstacle_detect_version', default_value='0',
            description='0: do not detect, 1: remove ground by fixed height, 2: remove groud by RANSAC, 3: remove groud by grid map'),

        DeclareLaunchArgument(
            'publish_low_obstacle_ground', default_value='false',
            description='publish ground to detect low obstacles only for debug purpose'),

        DeclareLaunchArgument(
            'footprint_publisher_version', default_value=EnvironmentVariable('CABOT_FOOTPRINT_PUBLISHER_VERSION', default_value='2'),
            description='Footprint publisher version'),

        # default navigator
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            respawn=True,
            respawn_delay=2.0,
            output=output,
            parameters=[configured_params],
            remappings=remappings,
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_controller',
                    output=output,
                    parameters=[
                        configured_params,
                        {
                            'autostart': autostart,
                            'node_names': [
                                'controller_server',
                            ]
                        },
                    ],
                ),
            ]
        ),

        # local odom navigator
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='local',
            respawn=True,
            respawn_delay=2.0,
            output=output,
            parameters=[configured_params2],
            remappings=remappings2,
            #            arguments=["--ros-args", "--log-level", "debug"]
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_local_controller',
                    output=output,
                    namespace='local',
                    parameters=[
                        configured_params2,
                        {
                            'autostart': autostart,
                            'node_names': [
                                'controller_server',
                            ]
                        },
                    ],
                )
            ]
        ),

    ])
