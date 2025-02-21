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
import tempfile
import traceback
import xml.dom.minidom

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess
from launch.event_handlers import OnShutdown
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import OrSubstitution
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.descriptions import ParameterValue
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from launch.logging import launch_config
from cabot_common.launch import AppendLogDirPrefix


class AddStatePlugin(Substitution):
    def __init__(self, source_file):
        super().__init__()
        self.source_file = normalize_to_list_of_substitutions(source_file)

    def describe(self):
        return ""

    def perform(self, context):
        xml_filename = perform_substitutions(context, self.source_file)
        rewritten_xml = tempfile.NamedTemporaryFile(mode='w', delete=False,
                                                    prefix='sdf', suffix='.xml')
        try:
            sdf = xml.dom.minidom.parse(xml_filename)
            plugin = xml.dom.minidom.parseString("""
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
  <ros>
    <namespace>/gazebo</namespace>
  </ros>
  <update_rate>1.0</update_rate>
</plugin>
            """)
            worlds = sdf.getElementsByTagName("world")
            if len(worlds) != 1:
                return xml_filename
            worlds[0].appendChild(plugin.firstChild)
            sdf.writexml(rewritten_xml)
            return rewritten_xml.name
        except:  # noqa: E722
            traceback.print_exc()
        return xml_filename


def generate_launch_description():
    pkg_dir = get_package_share_directory('cabot_gazebo')
    output = 'both'

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_name = LaunchConfiguration('model')
    world_file = LaunchConfiguration('world_file')
    wireless_config_file = LaunchConfiguration('wireless_config_file')
    gdb = LaunchConfiguration('gdb')

    gazebo_params = os.path.join(
        pkg_dir,
        "params/gazebo.params.yaml")

    check_gazebo_ready = Node(
        package='cabot_gazebo',
        executable='check_gazebo_ready.py',
        name='check_gazebo_ready_node',
    )

    use_livox = PythonExpression(['"', model_name, '" in ["cabot3-i1", "cabot3-m1", "cabot3-m2"]'])

    # these models have lidar attached side way
    lidar_target_frame = PythonExpression([
        '"lidar_base_link" if "', model_name,
        '" in ["cabot3-k1", "cabot3-k2", "cabot3-k4"]',
        'else "velodyne"']
    )

    xacro_for_cabot_model = PathJoinSubstitution([
        get_package_share_directory('cabot_description'),
        'robots',
        PythonExpression(['"', model_name, '.urdf.xacro.xml', '"'])
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_for_cabot_model, ' offset:=0.25', ' sim:=', use_sim_time]),
        value_type=str
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic',
            '/robot_description',
            '-entity',
            'mobile_base',
            '-x',
            EnvironmentVariable('CABOT_INITX', default_value='0'),
            '-y',
            EnvironmentVariable('CABOT_INITY', default_value='0'),
            '-z',
            PythonExpression(['str(0.01+', EnvironmentVariable('CABOT_INITZ', default_value='0'), ')']),
            '-Y',
            EnvironmentVariable('CABOT_INITAR', default_value='0')
        ]
    )

    modified_world = AddStatePlugin(world_file)

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot_gazebo")])),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='Robot URDF xacro model name'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value=PathJoinSubstitution([get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world']),
            description='Gazebo world file to be open'
        ),
        DeclareLaunchArgument(
            'wireless_config_file',
            default_value='',
            description='wireless config file'
        ),
        DeclareLaunchArgument(
            'gdb',
            default_value='false',
            description='use gdb'
        ),

        LogInfo(
            msg=['You need to specify model, world_file parameter'],
            condition=IfCondition(OrSubstitution(
                PythonExpression(['"', model_name, '"==""']),
                PythonExpression(['"', world_file, '"==""'])
            ))
        ),

        GroupAction([
            # publish robot state
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output=output,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'publish_frequency': 100.0,
                    'robot_description': robot_description
                }]
            ),
            # publish **local** robot state for local map navigation (getting off elevators)
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='local_robot_state_publisher',
                output=output,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'publish_frequency': 100.0,
                    'frame_prefix': 'local/',
                    'robot_description': robot_description
                }]
            ),

            # launch velodyne lider related nodes
            ComposableNodeContainer(
                name='laser_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[],
            ),

            LoadComposableNodes(
                target_container='/laser_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='pointcloud_to_laserscan',
                        plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                        namespace='',
                        name='pointcloud_to_laserscan_node',
                        parameters=[gazebo_params, {
                            'use_sim_time': use_sim_time,
                            'target_frame': lidar_target_frame
                        }],
                        remappings=[
                            ('/cloud_in', '/velodyne_points')
                        ]
                    ),
                    ComposableNode(
                        package='pcl_ros',
                        plugin='pcl_ros::CropBox',
                        namespace='',
                        name='filter_crop_box_node',
                        parameters=[gazebo_params, {'use_sim_time': use_sim_time}],
                        remappings=[
                            ('/input',  '/velodyne_points'),
                            ('/output', '/velodyne_points_cropped')
                        ]
                    ),
                ]
            ),

            # crop Livox point cloud only for simulator becasue Livox simulator has noise at close area
            ComposableNodeContainer(
                name='livox_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[],
            ),
            LoadComposableNodes(
                target_container='/livox_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='pcl_ros',
                        plugin='pcl_ros::CropBox',
                        namespace='',
                        name='livox_crop_box_node',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'min_x': -0.3,
                            'min_y': -0.3,
                            'min_z': -0.3,
                            'max_x': 0.3,
                            'max_y': 0.3,
                            'max_z': 0.3,
                            'keep_organized': False,
                            'negative': True,
                            'input_frame': 'livox',
                            'output_frame': 'livox',
                        }],
                        remappings=[
                            ('/input',  '/livox/lidar_PointCloud2'),
                            ('/output', '/livox/points')
                        ]
                    ),
                ],
                condition=IfCondition(use_livox)
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'),
                                              '/launch/gzserver.launch.py']),
                launch_arguments={
                    'gdb': gdb,
                    'verbose': 'true',
                    'world': modified_world,
                    'params_file': str(gazebo_params)
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_dir,
                                              '/launch/gazebo_wireless_helper.launch.py']),
                launch_arguments={
                    'verbose': 'false',
                    'namespace': 'wireless',
                    'wireless_config_file': wireless_config_file
                }.items(),
                condition=LaunchConfigurationNotEquals('wireless_config_file', '')
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_dir,
                                              '/launch/gazebo_pressure_helper.launch.py']),
                launch_arguments={
                    'verbose': 'true',
                    'namespace': 'pressure_simulator',
                    'pressure_topic': '/cabot/pressure',
                    'config_file': wireless_config_file,
                    'use_sim_time': use_sim_time,
                }.items(),
                condition=LaunchConfigurationNotEquals('wireless_config_file', '')
            ),

            # cabot feature handleside (ad-hoc implementation)
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'topic',
                    'pub',
                    '--qos-durability', 'transient_local',
                    '--qos-reliability', 'reliable',
                    '-1',
                    '--keep-alive', '9999999',
                    '/cabot/features/handleside',
                    'std_msgs/msg/String',
                    'data: left,right',
                ],
                output='screen',
            ),
            # cabot feature touchmode (ad-hoc implementation)
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'topic',
                    'pub',
                    '--qos-durability', 'transient_local',
                    '--qos-reliability', 'reliable',
                    '-1',
                    '--keep-alive', '9999999',
                    '/cabot/features/touchmode',
                    'std_msgs/msg/String',
                    'data: cap,tof,dual',
                ],
                output='screen',
            ),

            check_gazebo_ready,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=check_gazebo_ready,
                    on_exit=[
                        LogInfo(msg='Gazebo is ready'),
                        spawn_entity
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[
                        LogInfo(msg='Spawn finished')
                    ]
                )
            ),
        ],
            condition=UnlessCondition(OrSubstitution(
                PythonExpression(['"', model_name, '"==""']),
                PythonExpression(['"', world_file, '"==""'])
            ))
        )
    ])
