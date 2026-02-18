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

from launch.logging import launch_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

from cabot_common.launch import NamespaceParameterFile
from cabot_common.launch import GetPackageShareDirectory
from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    output = {'stderr': {'log'}}
    use_sim_time = LaunchConfiguration('use_sim_time')
    init_speed = LaunchConfiguration('init_speed')
    anchor_file = LaunchConfiguration('anchor_file')
    language = LaunchConfiguration('language')
    site = LaunchConfiguration('site')
    global_map_name = LaunchConfiguration('global_map_name')
    plan_topic = LaunchConfiguration('plan_topic')
    show_topology = LaunchConfiguration('show_topology')
    announce_no_touch = LaunchConfiguration('announce_no_touch')
    speed_poi_params = LaunchConfiguration('speed_poi_params')
    handle_button_mapping = LaunchConfiguration('handle_button_mapping')
    vibrator_type = LaunchConfiguration('vibrator_type')
    use_directional_indicator = LaunchConfiguration('use_directional_indicator')
    max_speed = LaunchConfiguration('max_speed')
    free_mode_detect_lidar_obstacles = LaunchConfiguration('free_mode_detect_lidar_obstacles')
    free_mode_detect_low_obstacles = LaunchConfiguration('free_mode_detect_low_obstacles')
    free_mode_detect_lidar_max_limit_speed = LaunchConfiguration('free_mode_detect_lidar_max_limit_speed')
    free_mode_detect_costmap_obstacles = LaunchConfiguration('free_mode_detect_costmap_obstacles')
    free_mode_detect_costmap_threshold = LaunchConfiguration('free_mode_detect_costmap_threshold')
    free_mode_detect_costmap_forward_distance = LaunchConfiguration('free_mode_detect_costmap_forward_distance')
    free_mode_warn_cost_threshold = LaunchConfiguration('free_mode_warn_cost_threshold')
    free_mode_warn_forward_distance = LaunchConfiguration('free_mode_warn_forward_distance')
    free_mode_warn_left_side_distance = LaunchConfiguration('free_mode_warn_left_side_distance')
    free_mode_warn_left_forward_distance = LaunchConfiguration('free_mode_warn_left_forward_distance')
    free_mode_warn_right_side_distance = LaunchConfiguration('free_mode_warn_right_side_distance')
    free_mode_warn_right_forward_distance = LaunchConfiguration('free_mode_warn_right_forward_distance')
    free_mode_warn_back_distance = LaunchConfiguration('free_mode_warn_back_distance')
    free_mode_stop_duration = LaunchConfiguration('free_mode_stop_duration')
    free_mode_correction_back_duration = LaunchConfiguration('free_mode_correction_back_duration')
    free_mode_correction_back_enabled = LaunchConfiguration('free_mode_correction_back_enabled')
    free_mode_correction_back_speed = LaunchConfiguration('free_mode_correction_back_speed')
    free_mode_correction_side_duration = LaunchConfiguration('free_mode_correction_side_duration')
    free_mode_correction_side_enabled = LaunchConfiguration('free_mode_correction_side_enabled')
    free_mode_correction_side_speed = LaunchConfiguration('free_mode_correction_side_speed')
    free_mode_correction_side_turnspeed = LaunchConfiguration('free_mode_correction_side_turnspeed')
    free_mode_correction_touch_required = LaunchConfiguration('free_mode_correction_touch_required')
    free_mode_switch_autonomous_mode = LaunchConfiguration('free_mode_switch_autonomous_mode')
    free_mode_switch_autonomous_wizard_mode = LaunchConfiguration('free_mode_switch_autonomous_wizard_mode')
    free_mode_switch_autonomous_mode_temp = LaunchConfiguration('free_mode_switch_autonomous_mode_temp')
    free_mode_switch_autonomous_mode_temp_duration = LaunchConfiguration('free_mode_switch_autonomous_mode_temp_duration')
    free_mode_end_userfree_movement_time = LaunchConfiguration('free_mode_end_userfree_movement_time')
    cabot_vlm_use_button = LaunchConfiguration('cabot_vlm_use_button')
    cabot_allowed_modes_bitmask = LaunchConfiguration('cabot_allowed_modes_bitmask')

    def hoge(text):
        return text

    config_path = PathJoinSubstitution([
        GetPackageShareDirectory(site),
        'config',
        'config.yaml'
    ])

    menu_file = PathJoinSubstitution([
        GetPackageShareDirectory('cabot_ui'),
        'config',
        'menu.yaml'
    ])

    default_param_file = PathJoinSubstitution([
        GetPackageShareDirectory('cabot_ui'),
        'config',
        'default_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('sigterm_timeout', default_value='15'),
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot_ui")])),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether the simulated time is used or not'
        ),
        DeclareLaunchArgument(
            'init_speed', default_value='',
            description='Set the robot initial maximum speed. This will be capped by the max speed.'
        ),
        DeclareLaunchArgument(
            'anchor_file', default_value='',
            description='File path that includes anchor data'
        ),
        DeclareLaunchArgument(
            'language',
            default_value=EnvironmentVariable('CABOT_LANG', default_value='en'),
            description='Set the system language'
        ),
        DeclareLaunchArgument(
            'site', default_value='',
            description='CaBot site package name'
        ),
        DeclareLaunchArgument(
            'global_map_name', default_value='map_global',
            description='Set the global map name'
        ),
        DeclareLaunchArgument(
            'plan_topic', default_value='/plan',
            description='Topic name for planned path'
        ),
        DeclareLaunchArgument(
            'show_topology', default_value='false',
            description='Show topology on rviz'
        ),
        DeclareLaunchArgument(
            'announce_no_touch',
            default_value=EnvironmentVariable('CABOT_ANNOUNCE_NO_TOUCH', default_value='false'),
            description='True if you want to announce detection of no touch'
        ),
        DeclareLaunchArgument(
            'speed_poi_params',
            default_value=EnvironmentVariable('CABOT_SPEED_POI_PARAMS', default_value='[0.5, 0.5, 0.5]'),
            description='[target_distance, expected_deceleration, expected_delay]'
        ),
        DeclareLaunchArgument(
            'handle_button_mapping',
            default_value=EnvironmentVariable('CABOT_HANDLE_BUTTON_MAPPING', default_value='2'),
            description="Set the handle button mapping"
        ),
        DeclareLaunchArgument(
            'vibrator_type',
            default_value=EnvironmentVariable('CABOT_VIBRATOR_TYPE', default_value='1'),
            description='1: ERM (Eccentric Rotating Mass), 2: LRA (Linear Resonant Actuator)'
        ),
        DeclareLaunchArgument(
            'use_directional_indicator',
            default_value=EnvironmentVariable('CABOT_USE_DIRECTIONAL_INDICATOR', default_value='false'),
            description='If true, the directional indicator on the handle is enabled'
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value=EnvironmentVariable('CABOT_MAX_SPEED', default_value='1.0'),
            description='Set maximum speed of the robot'
        ),
        DeclareLaunchArgument(
            'free_mode_detect_lidar_obstacles',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_DETECT_LIDAR_OBSTACLES', default_value='false')
        ),
        DeclareLaunchArgument(
            'free_mode_detect_low_obstacles',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_DETECT_LOW_OBSTACLES', default_value='false')
        ),
        DeclareLaunchArgument(
            'free_mode_detect_lidar_max_limit_speed',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_DETECT_LIDAR_MAX_LIMIT_SPEED', default_value='0.05')
        ),
        DeclareLaunchArgument(
            'free_mode_detect_costmap_obstacles',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_DETECT_COSTMAP_OBSTACLES', default_value='true')
        ),
        DeclareLaunchArgument(
            'free_mode_detect_costmap_threshold',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_DETECT_COSTMAP_THRESHOLD', default_value='30')
        ),
        DeclareLaunchArgument(
            'free_mode_detect_costmap_forward_distance',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_DETECT_COSTMAP_FORWARD_DISTANCE', default_value='0.5')
        ),
        DeclareLaunchArgument(
            'free_mode_warn_cost_threshold',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_WARN_COST_THRESHOLD', default_value='40'),
        ),
        DeclareLaunchArgument(
            'free_mode_warn_forward_distance',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_WARN_FORWARD_DISTANCE', default_value='0.7'),
        ),
        DeclareLaunchArgument(
            'free_mode_warn_left_side_distance',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_WARN_LEFT_SIDE_DISTANCE', default_value='0.3'),
        ),
        DeclareLaunchArgument(
            'free_mode_warn_left_forward_distance',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_WARN_LEFT_FORWARD_DISTANCE', default_value='0.3'),
        ),
        DeclareLaunchArgument(
            'free_mode_warn_right_side_distance',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_WARN_RIGHT_SIDE_DISTANCE', default_value='0.3'),
        ),
        DeclareLaunchArgument(
            'free_mode_warn_right_forward_distance',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_WARN_RIGHT_FORWARD_DISTANCE', default_value='0.3'),
        ),
        DeclareLaunchArgument(
            'free_mode_warn_back_distance',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_WARN_BACK_DISTANCE', default_value='0.3'),
        ),
        DeclareLaunchArgument(
            'free_mode_stop_duration',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_STOP_DURATION', default_value='0.5'),
        ),
        DeclareLaunchArgument(
            'free_mode_correction_back_duration',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_CORRECTION_BACK_DURATION', default_value='0.5'),
        ),
        DeclareLaunchArgument(
            'free_mode_correction_back_enabled',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_CORRECTION_BACK_ENABLED', default_value='true'),
        ),
        DeclareLaunchArgument(
            'free_mode_correction_back_speed',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_CORRECTION_BACK_SPEED', default_value='0.2'),
        ),
        DeclareLaunchArgument(
            'free_mode_correction_side_duration',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_CORRECTION_SIDE_DURATION', default_value='0.5'),
        ),
        DeclareLaunchArgument(
            'free_mode_correction_side_enabled',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_CORRECTION_SIDE_ENABLED', default_value='true'),
        ),
        DeclareLaunchArgument(
            'free_mode_correction_side_speed',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_CORRECTION_SIDE_SPEED', default_value='0.2'),
        ),
        DeclareLaunchArgument(
            'free_mode_correction_side_turnspeed',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_CORRECTION_SIDE_TURNSPEED', default_value='0.5'),
        ),
        DeclareLaunchArgument(
            'free_mode_correction_touch_required',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_CORRECTION_TOUCH_REQUIRED', default_value='true'),
        ),
        DeclareLaunchArgument(
            'free_mode_switch_autonomous_mode',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_SWITCH_AUTONOMOUS_MODE', default_value='false'),
        ),
        DeclareLaunchArgument(
            'free_mode_switch_autonomous_wizard_mode',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_SWITCH_AUTONOMOUS_WIZARD_MODE', default_value='false'),
        ),
        DeclareLaunchArgument(
            'free_mode_switch_autonomous_mode_temp',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_SWITCH_AUTONOMOUS_MODE_TEMP', default_value='false'),
        ),
        DeclareLaunchArgument(
            'free_mode_switch_autonomous_mode_temp_duration',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_SWITCH_AUTONOMOUS_MODE_TEMP_DURATION', default_value='5.0'),
        ),
        DeclareLaunchArgument(
            'free_mode_end_userfree_movement_time',
            default_value=EnvironmentVariable('CABOT_FREE_MODE_END_USERFREE_MOVEMENT_TIME', default_value='0.5'),
        ),
        DeclareLaunchArgument(
            'cabot_vlm_use_button',
            default_value=EnvironmentVariable('CABOT_VLM_USE_BUTTON', default_value='false'),
        ),
        DeclareLaunchArgument(
            'cabot_allowed_modes_bitmask',
            default_value=EnvironmentVariable('CABOT_ALLOWED_MODES_BITMASK', default_value='15'),
        ),
        Node(
            package="cabot_ui",
            executable="cabot_ui_manager.py",
            output=output,
            # name="cabot_ui_manager",  # must not set name because it makes multiple nodes with different names
            parameters=[{
                'use_sim_time': use_sim_time,
                'init_speed': init_speed,
                'anchor_file': anchor_file,
                'language': language,
                'global_map_name': global_map_name,
                'plan_topic': plan_topic,
                'menu_file': menu_file,
                'speed_poi_params': speed_poi_params,
                'handle_button_mapping': handle_button_mapping,
                'max_speed': max_speed,
                'free_mode_detect_lidar_obstacles': free_mode_detect_lidar_obstacles,
                'free_mode_detect_low_obstacles': free_mode_detect_low_obstacles,
                'free_mode_detect_lidar_max_limit_speed': free_mode_detect_lidar_max_limit_speed,
                'free_mode_detect_costmap_obstacles': free_mode_detect_costmap_obstacles,
                'free_mode_detect_costmap_threshold': free_mode_detect_costmap_threshold,
                'free_mode_detect_costmap_forward_distance': free_mode_detect_costmap_forward_distance,
                'free_mode_warn_cost_threshold': free_mode_warn_cost_threshold,
                'free_mode_warn_forward_distance': free_mode_warn_forward_distance,
                'free_mode_warn_left_side_distance': free_mode_warn_left_side_distance,
                'free_mode_warn_left_forward_distance': free_mode_warn_left_forward_distance,
                'free_mode_warn_right_side_distance': free_mode_warn_right_side_distance,
                'free_mode_warn_right_forward_distance': free_mode_warn_right_forward_distance,
                'free_mode_warn_back_distance': free_mode_warn_back_distance,
                'free_mode_stop_duration': free_mode_stop_duration,
                'free_mode_correction_back_duration': free_mode_correction_back_duration,
                'free_mode_correction_back_enabled': free_mode_correction_back_enabled,
                'free_mode_correction_back_speed': free_mode_correction_back_speed,
                'free_mode_correction_side_duration': free_mode_correction_side_duration,
                'free_mode_correction_side_enabled': free_mode_correction_side_enabled,
                'free_mode_correction_side_speed': free_mode_correction_side_speed,
                'free_mode_correction_side_turnspeed': free_mode_correction_side_turnspeed,
                'free_mode_correction_touch_required': free_mode_correction_touch_required,
                'free_mode_switch_autonomous_mode': free_mode_switch_autonomous_mode,
                'free_mode_switch_autonomous_wizard_mode': free_mode_switch_autonomous_wizard_mode,
                'free_mode_switch_autonomous_mode_temp': free_mode_switch_autonomous_mode_temp,
                'free_mode_switch_autonomous_mode_temp_duration': free_mode_switch_autonomous_mode_temp_duration,
                'free_mode_end_userfree_movement_time': free_mode_end_userfree_movement_time,
                'cabot_vlm_use_button': cabot_vlm_use_button,
                'cabot_allowed_modes_bitmask': cabot_allowed_modes_bitmask,
            }, NamespaceParameterFile('cabot_ui_manager_navigation', config_path)],
            ros_arguments=[
                # '--log-level', 'cabot_ui_manager:=debug'
            ],
            # prefix='python3 -m cProfile -o cabot_ui_manager.profile',
        ),
        Node(
            package='cabot_ui',
            executable='navcog_map.py',
            namespace='cabot',
            name='navcog_map',
            output=output,
            parameters=[{
                'anchor_file': anchor_file,
            }, NamespaceParameterFile('cabot/navcog_map', config_path)],
            condition=IfCondition(show_topology),
        ),
        Node(
            package='parameter_server',
            executable='server',
            arguments=[
                '--allow-declare', 'false',
                '--file-path', '/home/developer/ros2_ws/persistent_params.yaml',
            ],
            output=output,
            parameters=[default_param_file],
        ),
        Node(
            package="cabot_ui",
            executable="stop_reasons_node",
            name="stop_reasons_node",
            output=output,
            parameters=[{
                'announce_no_touch': announce_no_touch
            }],
        ),
        Node(
            package="cabot_common",
            executable="lookup_transform_service_node",
            name="lookup_transform_service_node",
            output=output,
        ),
        Node(
            package='cabot_ui',
            executable='cabot_handle_v2_node',
            namespace='/cabot',
            name='cabot_handle_v2_node',
            output=output,
            parameters=[
                default_param_file,
                {
                    'use_sim_time': use_sim_time,
                }
            ],
            condition=UnlessCondition(use_directional_indicator),
        ),
        Node(
            package='cabot_ui',
            executable='cabot_handle_v3_node',
            namespace='/cabot',
            name='cabot_handle_v3_node',
            output=output,
            parameters=[
                default_param_file,
                {
                    'use_sim_time': use_sim_time,
                    'vibrator_type': vibrator_type
                }
            ],
            condition=IfCondition(use_directional_indicator),
        ),
    ])
