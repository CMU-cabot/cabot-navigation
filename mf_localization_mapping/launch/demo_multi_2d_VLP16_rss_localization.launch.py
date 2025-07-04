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
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.descriptions import ParameterValue
from launch.utilities import normalize_to_list_of_substitutions


def generate_launch_description():
    pkg_dir = get_package_share_directory('mf_localization_mapping')
    mf_localization_dir = get_package_share_directory('mf_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot = LaunchConfiguration('robot')
    cabot_model = LaunchConfiguration('cabot_model')

    convert_points = LaunchConfiguration('convert_points')
    convert_imu = LaunchConfiguration('convert_imu')
    convert_esp32 = LaunchConfiguration('convert_esp32')

    # multi-floor manager
    map_config_file = LaunchConfiguration('map_config_file')
    tags = LaunchConfiguration('tags')
    beacon_topic = LaunchConfiguration('beacon_topic')
    wifi_topic = LaunchConfiguration('wifi_topic')
    rssi_offset = LaunchConfiguration('rssi_offset')
    pressure_topic = LaunchConfiguration('pressure_topic')
    pressure_available = LaunchConfiguration('pressure_available')
    verbose = LaunchConfiguration('verbose')
    use_gnss = LaunchConfiguration('use_gnss')
    use_global_localizer = LaunchConfiguration('use_global_localizer')
    gnss_fix = LaunchConfiguration('gnss_fix')
    gnss_fix_velocity = LaunchConfiguration('gnss_fix_velocity')
    navsat = LaunchConfiguration('navsat')
    # cartographer
    scan = LaunchConfiguration('scan')
    imu = LaunchConfiguration('imu')
    odom = LaunchConfiguration('odom')
    points2 = LaunchConfiguration('points2')
    imu_temp = LaunchConfiguration('imu_temp')
    points2_temp = LaunchConfiguration('points2_temp')

    # rosbag
    playbag = LaunchConfiguration('playbag')
    bag_filename = LaunchConfiguration('bag_filename')
    rate = LaunchConfiguration('rate')
    start_time = LaunchConfiguration('start_time')

    record_bag = LaunchConfiguration('record_bag')
    record_file = LaunchConfiguration('record_file')

    # debug
    log_level = LaunchConfiguration("log_level")

    # rviz
    # site = LaunchConfiguration('site')  # not used

    # run multi_floor_manager
    multi_floor_config_filename = LaunchConfiguration('multi_floor_config_filename')

    # robot localization
    compute_odometry_filtered = LaunchConfiguration('compute_odometry_filtered')
    republish_odometry_filtered = LaunchConfiguration('republish_odometry_filtered')

    param_files = [
        ParameterFile(
            PathJoinSubstitution([
                mf_localization_dir,
                'configuration_files',
                'multi_floor',
                multi_floor_config_filename,
            ]),
            allow_substs=True,
        ),
        ParameterFile(PathJoinSubstitution([
            pkg_dir,
            'configuration_files',
            'demo_config.yaml'
        ]),
            allow_substs=True,
        )
    ]

    def configure_ros2_bag_play(context, node):
        cmd = node.cmd.copy()
        cmd.extend(['--clock', '--start-paused', '--rate', rate])
        if float(start_time.perform(context)) > 0.0:
            cmd.extend(['--start-offset', start_time])
        cmd.extend([
            '--remap',
            'imu/data:=imu/data',
            'tf:=tf_temp',
            'tf_static:=tf_static_temp',
            # odometry
            'cabot/odometry/filtered:=cabot/odometry/filtered_temp',
            'odom:=odom_temp',
            # mf_localization
            'current_floor:=current_floor_temp',
            'current_floor_raw:=current_floor_raw_temp',
            'current_floor_smoothed:=current_floor_smoothed_temp',
            'current_frame:=current_frame_temp',
            'current_map_filename:=current_floor_map_filename_temp',
            'current_area:=current_area_temp',
            'current_mode:=current_mode_temp',
            'localize_status:=localize_status_temp',
            'map:=map_temp',
            # ublox_converter
            'mf_navsat:=mf_navsat_temp',
            'ublox_converter/num_active_sv:=ublox_converter/num_active_sv_temp',
            'ublox_converter/sv_status:=ublox_converter/sv_status_temp',
        ])
        if convert_points.perform(context) == 'true':
            cmd.append([points2, ':=', points2_temp])
        cmd.extend(['--', bag_filename])
        node.cmd.clear()
        # needs to be normalized
        node.cmd.extend([normalize_to_list_of_substitutions(x) for x in cmd])
        return [node]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        DeclareLaunchArgument('robot', default_value='rover'),
        DeclareLaunchArgument('cabot_model', default_value=''),

        DeclareLaunchArgument('convert_points', default_value='false'),
        DeclareLaunchArgument('convert_imu', default_value='false'),
        DeclareLaunchArgument('convert_esp32', default_value='false'),

        # multi-floor manager
        DeclareLaunchArgument('map_config_file'),
        DeclareLaunchArgument('tags', default_value=''),
        DeclareLaunchArgument('beacon_topic', default_value='beacons'),
        DeclareLaunchArgument('wifi_topic', default_value='wifi'),
        DeclareLaunchArgument('rssi_offset', default_value=''),
        DeclareLaunchArgument('pressure_topic', default_value='pressure'),
        DeclareLaunchArgument('pressure_available', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_gnss', default_value='false'),
        DeclareLaunchArgument('use_global_localizer', default_value='false'),
        DeclareLaunchArgument('gnss_fix', default_value='gnss_fix'),
        DeclareLaunchArgument('gnss_fix_velocity', default_value='gnss_fix_velocity'),
        DeclareLaunchArgument('navsat', default_value='ublox/navsat'),
        # cartographer
        DeclareLaunchArgument('scan', default_value='velodyne_scan'),
        DeclareLaunchArgument('imu', default_value='imu/data'),
        DeclareLaunchArgument('odom', default_value='odom'),
        DeclareLaunchArgument('points2', default_value='velodyne_points'),
        DeclareLaunchArgument('imu_temp', default_value='imu_temp/data'),
        DeclareLaunchArgument('points2_temp', default_value='velodyne_points_temp'),

        # rosbag
        DeclareLaunchArgument('playbag', default_value='true'),
        DeclareLaunchArgument('bag_filename'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('start_time', default_value='0.0'),

        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('record_file', default_value=PythonExpression(['"', bag_filename, '"+".play.bag"'])),

        # rviz
        DeclareLaunchArgument('site', default_value=''),

        DeclareLaunchArgument('multi_floor_config_filename',
                              default_value='multi_floor_manager.yaml'),

        # robot_localization
        DeclareLaunchArgument('compute_odometry_filtered', default_value='false'),

        DeclareLaunchArgument('republish_odometry_filtered', default_value='false'),

        # debug
        DeclareLaunchArgument('log_level', default_value='info', description="logging level"),

        SetParameter('use_sim_time', use_sim_time),

        # convert esp32 wifi_scan_str
        Node(
            package='wireless_scanner_ros',
            executable='esp32_wifi_scan_converter.py',
            name='esp32_wifi_scan_converter',
            condition=IfCondition(convert_esp32),
            remappings=[('wireless/wifi', 'esp32/wifi')],
        ),

        # map server
        # IncludeLaunchDescription(
        #    AnyLaunchDescriptionSource(
        #        PathJoinSubstitution([get_package_share_directory('cabot_common', '', 'multi_floor_map_server.launch.xml'])
        #    ),
        #    launch_arguments={
        #        "map_config_file": map_config_file
        #    }.items()
        # ),

        # map loader
        Node(
            package='cabot_common',
            executable='map_loader.py',
            name='map_loader_node'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': ''}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server']}],
        ),

        # run lookup_transform_service_node for BufferProxy in multi_floor_manager
        Node(
            package="cabot_common",
            executable="lookup_transform_service_node",
            name="lookup_transform_service_node",
        ),

        # [deprecated] run ublox_converter
        # Node(
        #     package='mf_localization',
        #     executable='ublox_converter.py',
        #     name='ublox_converter',
        #     parameters=[
        #         PathJoinSubstitution([mf_localization_dir, 'configuration_files', 'ublox/ublox_converter.yaml']),
        #         {
        #             'use_sim_time': use_sim_time
        #         }
        #     ],
        #     remappings=[
        #         ('navsat', 'ublox/navsat'),
        #         ('num_active_sv', 'ublox_converter/num_active_sv'),
        #         ('sv_status', 'ublox_converter/sv_status')
        #     ],
        # ),

        # run multi_floor_manager
        GroupAction([
            SetParameter('robot', robot, condition=LaunchConfigurationNotEquals('robot', '')),
            SetParameter('rssi_offset', rssi_offset, condition=LaunchConfigurationNotEquals('rssi_offset', '')),
            Node(
                package='mf_localization',
                executable='multi_floor_topic_proxy',
                name='multi_floor_topic_proxy',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'map_config_file': map_config_file,
                    'verbose': True,
                }],
                remappings=[
                    ('points2', points2),
                    ('imu', imu),
                ],
                condition=IfCondition("true"),
            ),
            Node(
                package='mf_localization',
                executable='multi_floor_manager.py',
                name='multi_floor_manager',
                parameters=[
                    *param_files,
                    {
                        'use_sim_time': use_sim_time,
                        'map_config_file': map_config_file,
                        'tags': tags,
                        'configuration_directory': PathJoinSubstitution([mf_localization_dir, 'configuration_files', 'cartographer']),
                        'configuration_file_prefix': 'cartographer_2d',
                        'pressure_available': pressure_available,
                        'verbose': verbose,
                        'use_gnss': use_gnss,
                        'use_global_localizer': use_global_localizer,
                    }
                ],
                remappings=[
                    ('beacons', beacon_topic),
                    ('wifi', wifi_topic),
                    ('points2', points2),
                    ('imu', imu),
                    ('odom', odom),
                    ('scan', scan),
                    ('pressure', pressure_topic),
                    ('gnss_fix', gnss_fix),
                    ('gnss_fix_velocity', gnss_fix_velocity),
                    # ublox_converter
                    ('navsat', navsat),
                    ('num_active_sv', 'ublox_converter/num_active_sv'),
                    ('sv_status', 'ublox_converter/sv_status')
                ],
                output={},
                arguments=['--ros-args', '--log-level', PythonExpression(['"multi_floor_manager:=" + "', log_level, '"'])]
            ),
        ]),

        # publish robot model
        # use cabot_model argument if specified
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_frequency': 100.0,
                'robot_description': ParameterValue(
                    # xacro cabot_description/share/cabot_description/robots/cabot_model.urdf.xacro.xml sim:=use_sim_time
                    Command(['xacro ',
                             PathJoinSubstitution([
                                get_package_share_directory('cabot_description'),
                                'robots',
                                PythonExpression(['"', cabot_model, '.urdf.xacro.xml', '"'])
                                ]),
                            ' sim:=', use_sim_time]),
                    value_type=str
                )
            }],
            condition=LaunchConfigurationNotEquals("cabot_model", "")
        ),
        # use urdf in pkg_dir if cabot_model is not specified
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_frequency': 100.0,
                'robot_description': ParameterValue(
                    Command(['cat ', pkg_dir, '/urdf/', robot, '.urdf']),
                    value_type=str
                )
            }],
            condition=LaunchConfigurationEquals("cabot_model", "")
        ),

        # robot localization ekf_node to compute /cabot/odometry/filtered from odom_raw and imu data
        Node(
            package='robot_localization',
            executable='ekf_node',
            namespace='/cabot',
            name='ekf_node',
            parameters=[
                *param_files,
                {
                    'use_sim_time': use_sim_time
                }
            ],
            condition=IfCondition(compute_odometry_filtered),
        ),

        # nodes for republishing /cabot/odometry/filtered topic recorded in bag file
        # convert child_frame_id in /cabot/odometry/filtered topic to base_control_shift
        Node(
            package='mf_localization',
            executable='odometry_frame_converter_node',
            name='odometry_frame_converter_node',
            parameters=[
                {
                    'use_sim_time': use_sim_time
                }
            ],
            remappings=[
                ('odom_in', '/cabot/odometry/filtered_temp'),
                ('odom_out', '/cabot/odometry/filtered'),
            ],
            condition=IfCondition(republish_odometry_filtered),
        ),
        # republish /tf (odom -> base_footprint -> base_control_shift)
        Node(
            package='mf_localization',
            executable='tf_selector_node',
            name='tf_selector_node',
            parameters=[
                {
                    'use_sim_time': use_sim_time
                }
            ],
            remappings=[
                ('tf_in', 'tf_temp'),
                ('tf_out', 'tf'),
            ],
            condition=IfCondition(republish_odometry_filtered),
        ),

        # /velodyne_packets to /velodyne_points
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([pkg_dir, 'launch', 'includes', 'VLP16_points_cloud_nodelet_cartographer.launch.py'])
            ),
            condition=IfCondition(convert_points),
            launch_arguments={
                'scan': scan
            }.items()
        ),

        # rename imu frame_id
        Node(
            package='mf_localization',
            executable='imu_frame_renamaer.py',
            name='imu_frame_renamer',
            condition=IfCondition(convert_imu),
            remappings=[('imu_in', imu_temp), ('imu_out', imu)],
        ),

        # play
        OpaqueFunction(
            function=configure_ros2_bag_play,
            args=[ExecuteProcess(
                cmd=['xterm', '-e', 'ros2', 'bag', 'play'],
                condition=IfCondition(playbag),
            )]
        ),

        # record
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', record_file],
            condition=IfCondition(record_bag),
        ),

        # rviz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', [pkg_dir, '/configuration_files/rviz/demo_2d_floors.rviz']],
        ),
    ])
