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
            'autostart', default_value='false',
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
            'low_obstacle_detect_version', default_value='2',
            description='0: do not detect, 1: remove ground by fixed height, 2: remove groud by RANSAC, 3: remove groud by grid map'),

        DeclareLaunchArgument(
            'publish_low_obstacle_ground', default_value='true',
            description='publish ground to detect low obstacles only for debug purpose'),

        DeclareLaunchArgument(
            'footprint_publisher_version', default_value=EnvironmentVariable('CABOT_FOOTPRINT_PUBLISHER_VERSION', default_value='2'),
            description='Footprint publisher version'),

        # low obstacle detection
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            namespace='',
            name='livox_pointcloud_to_laserscan_node',
            output=output,
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'livox_footprint',
                'transform_tolerance': 0.01,
                'angle_min': -0.614,  # -35.2*M_PI/180
                'angle_max': 0.614,  # 35.2*M_PI/180
                'angle_increment': 0.00174,  # M_PI/180/10
                'scan_time': 0.1,
                'range_min': 0.05,
                'range_max': 5.0,  # must be greater than or equal to max_range parameters in low_obstacle_layer
                'use_inf': True,
                'inf_epsilon': 1.0,
                # Concurrency level affects number of pointclouds queued for
                # processing and number of threads used
                # 0 : Detect number of cores
                # 1 : Single threaded
                # 2->inf : Parallelism level
                'concurrency_level': 0
            }],
            remappings=[
                ('/cloud_in', '/livox/points_filtered'),
                ('/scan', '/livox_scan')
            ],
            condition=IfCondition(use_low_obstacle_detect)
        ),


        Node(
            package='cabot_navigation2',
            executable='limit_fov_scan_expand',
            namespace='',
            name='limit_fov_scan_expand_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/livox_scan',
                'output_topic': '/livox_scan_expand',
                'expand_angle': 1.0
            }],
            condition=IfCondition(use_low_obstacle_detect)
        ),

        Node(
            package='cabot_navigation2',
            executable='ransac_ground_filter_node',
            namespace='',
            name='ransac_ground_filter_node',
            output=output,
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'livox_footprint',
                'min_range': 0.05,
                'max_range': 5.0,
                'min_height': -1.8,
                'max_height': 1.8,
                'publish_debug_ground': publish_low_obstacle_ground,
                'output_debug_ground_topic': '/ground_filter_ground',
                'ground_distance_threshold': 0.05,
                'xfer_format': 0,
                'ignore_noise': True,
                'input_topic': '/livox/points',
                'output_ground_topic': '/livox/points_ground',
                'output_filtered_topic': '/livox/points_filtered',
                'ransac_max_iteration': 10000,
                'ransac_probability': 0.999,
                'ransac_eps_angle': 5.0,
                'ransac_input_min_height': -0.50,
                'ransac_input_max_height': 0.50,
                'ransac_inlier_threshold': 0.01
            }],
            condition=IfCondition(PythonExpression([low_obstacle_detect_version, " == 2"]))
        ),

    ])
