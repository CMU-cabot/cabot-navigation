#!/usr/bin/env python3
# Copyright (c) 2021, 2023  IBM Corporation and Carnegie Mellon University

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch.utilities import normalize_to_list_of_substitutions


def generate_launch_description():
    pkg_dir = get_package_share_directory('mf_localization_mapping')

    robot = LaunchConfiguration('robot')
    # topic
    scan = LaunchConfiguration('scan')
    imu = LaunchConfiguration('imu')
    points2 = LaunchConfiguration('points2')
    fix = LaunchConfiguration('fix')
    # config
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    save_state_filename = LaunchConfiguration('save_state_filename')
    load_state_filename = LaunchConfiguration('load_state_filename')
    start_trajectory_with_default_topics = LaunchConfiguration('start_trajectory_with_default_topics')
    # config for post processing
    convert_pbstream = LaunchConfiguration('convert_pbstream')
    save_state_directory = LaunchConfiguration('save_state_directory')
    save_state_filestem = LaunchConfiguration('save_state_filestem')

    # function to configure a node command argument
    def add_cartographer_arguments(context, node):
        cmd = node.cmd.copy()
        if save_state_filename.perform(context) != "":
            cmd.insert(1, save_state_filename)
            cmd.insert(1, "-save_state_filename")
        if load_state_filename.perform(context) != "":
            cmd.insert(1, load_state_filename)
            cmd.insert(1, "-load_state_filename")
        node.cmd.clear()
        # needs to be normalized
        node.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd])
        return [node,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=node,
                        on_exit=[
                            convert_pbstream_to_pgm
                        ]
                    ),
                    condition=IfCondition(convert_pbstream)
                ),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=convert_pbstream_to_pgm,
                        on_exit=[
                            convert_pgm_to_png
                        ]
                    ),
                    condition=IfCondition(convert_pbstream)
                ),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=convert_pgm_to_png,
                        on_exit=[
                            extract_floor_map_info
                        ]
                    ),
                    condition=IfCondition(convert_pbstream)
                ),
                ]

    # post process commands
    convert_pbstream_to_pgm = ExecuteProcess(
       cmd=["ros2", "run", "cartographer_ros", "cartographer_pbstream_to_ros_map",
            "-pbstream_filename", [save_state_filestem, ".pbstream"],
            '-map_filestem', save_state_filestem],
       cwd=save_state_directory
    )

    convert_pgm_to_png = ExecuteProcess(
        cmd=["convert", [save_state_filestem, ".pgm"], [save_state_filestem, ".png"]],
        cwd=save_state_directory
    )

    extract_floor_map_info = ExecuteProcess(
        cmd=["ros2", "run", "mf_localization_mapping", "extract_floormap_info_from_yaml.py",
             "--input", [save_state_filestem, ".yaml"],
             "--output", [save_state_filestem, ".info.txt"],
             "&&"
             "identify", "-format", "'width: %w\\nheight: %h\\n'", [save_state_filestem, ".pgm"],
             "|",
             "tee", "-a", [save_state_filestem, ".info.txt"]
             ],
        cwd=save_state_directory,
        shell=True
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='rover'),
        # topic
        DeclareLaunchArgument('scan', default_value='velodyne_scan'),
        DeclareLaunchArgument('imu', default_value='imu/data'),
        DeclareLaunchArgument('points2', default_value='velodyne_points'),
        DeclareLaunchArgument('fix', default_value='fix'),
        # config
        DeclareLaunchArgument('configuration_directory', default_value=PathJoinSubstitution([pkg_dir, 'configuration_files', 'cartographer'])),
        DeclareLaunchArgument('configuration_basename', default_value='cartographer_2d_mapping.lua'),
        DeclareLaunchArgument('save_state_filename', default_value=''),
        DeclareLaunchArgument('load_state_filename', default_value=''),
        DeclareLaunchArgument('start_trajectory_with_default_topics', default_value='true'),
        # config for post processing
        DeclareLaunchArgument('convert_pbstream', default_value='false'),
        DeclareLaunchArgument('save_state_directory', default_value=''),
        DeclareLaunchArgument('save_state_filestem', default_value=''),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', pkg_dir, '/urdf/', robot, '.urdf']),
                    value_type=str
                )
            }]
        ),

        # wrap with a OpaqueFunction to enable conditional arguments which is not possible with usual way
        # https://answers.ros.org/question/314903/ros2-optionally-launch-node-with-no-arguments/
        # Each item of Node's arguments is tread as a string, so null string can be passed as an argument
        OpaqueFunction(
            function=add_cartographer_arguments,
            args=[Node(
                name="cartographer_node",
                package="cartographer_ros",
                executable="cartographer_node",
                arguments=[
                    "-configuration_directory", configuration_directory,
                    "-configuration_basename", configuration_basename,
                    ["-start_trajectory_with_default_topics=", start_trajectory_with_default_topics]
                ],
                remappings=[
                    ("scan", scan),
                    ("points2", points2),
                    ("imu", imu),
                    ("fix", fix),
                ]
            )]
        ),

        Node(
            package="mf_localization",
            executable="trajectory_restarter.py",
            name="trajectory_restarter",
            parameters=[{
                "configuration_directory": configuration_directory,
                "configuration_basename": configuration_basename
            }]
        ),

        Node(
            name="cartographer_occupancy_grid_node",
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            arguments=["-resolution", "0.015"]
        ),
    ])
