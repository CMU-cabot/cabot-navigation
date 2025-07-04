#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025 IBM Corporation
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


def generate_launch_description():
    output = {'stderr': {'log'}}
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    config_file = LaunchConfiguration('config_file')
    pressure_topic = LaunchConfiguration('pressure_topic')
    verbose = LaunchConfiguration('verbose')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time'),

        DeclareLaunchArgument(
            'namespace',
            description='Namespace of wireless nodes'),

        DeclareLaunchArgument(
            'config_file',
            description='Config file'),

        DeclareLaunchArgument(
            'pressure_topic',
            description='Pressure topic'),

        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='True if output more'),

        Node(
            package='cabot_gazebo',
            executable='pressure_simulator_node.py',
            name='pressure_simulator',
            namespace=namespace,
            output=output,
            parameters=[{
                'use_sim_time': use_sim_time,
                'config_file': config_file,
                'verbose': verbose
            }],
            remappings=[
                ("pressure", pressure_topic)
            ]
        ),
    ])
