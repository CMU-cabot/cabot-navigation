# Copyright (c) 2022, 2023  Carnegie Mellon University
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

# Launch file for CaBot control (model independent)

from launch.logging import launch_config

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from cabot_common.launch import AppendLogDirPrefix


def generate_launch_description():
    output = 'both'
    pkg_dir = get_package_share_directory('cabot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    touch_enabled = LaunchConfiguration('touch_enabled')
    max_speed = LaunchConfiguration('max_speed')

    param_files = [
        ParameterFile(PathJoinSubstitution([
            pkg_dir,
            'config',
            'cabot-control.yaml'
        ]),
            allow_substs=True,
        )
    ]

    # deprecated parameters
    # - offset
    # - no_vibration
    # - output
    # - use_velodyne
    # - use_tf_static

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether the simulated time is used or not'
        ),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(
            OnShutdown(on_shutdown=[AppendLogDirPrefix("cabot")]),
            condition=UnlessCondition(use_sim_time)
        ),
        DeclareLaunchArgument(
            'touch_enabled',
            default_value=EnvironmentVariable('CABOT_TOUCH_ENABLED', default_value='true'),
            description='If true, the touch sensor on the handle is used to control speed'
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value=EnvironmentVariable('CABOT_MAX_SPEED', default_value='1.0'),
            description='Set maximum speed of the robot'
        ),

        # Visualize the current speed on Rviz-
        Node(
            package='cabot',
            executable='speed_visualize_node',
            namespace='/cabot',
            name='speed_visualize_node',
            output=output,
            parameters=[*param_files, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='cabot',
            executable='wheelie_control_node',
            namespace='/cabot',
            name='wheelie_control_node',
            output=output,
            parameters=[*param_files, {'use_sim_time': use_sim_time}],
        ),


        # Costmap clearing issue hacking
        # Some obstacle points in costmap can be laid between the line of sight of lasers.
        # This requires the robot to move to clear those points. Usually this problem is
        # dealed with the rotating recovery behavior[1] in default recovery behaviors,
        # but this behavior is removed for CaBot because rotation is annoying for the user.
        # [1] https://github.com/ros-planning/navigation/tree/kinetic-devel/rotate_recovery
        # So, this node randomly rotate the laser in range of a laser scan step
        # (360/1440 degree) by changing hokuyo_link tf to remove obstacle points between
        # two laser scans.
        Node(  # TODO use component
            package='cabot',
            executable='clearing_tf_node',
            namespace='/cabot',
            name='clearing_tf_node',
            output=output,
            parameters=[*param_files, {'use_sim_time': use_sim_time}],
        ),

        # The diagram of Cabot Odometry Adapter & related nodes (*components)
        # move_base's cmd_vel commands will be filtered through nodes to transform
        # command for the robot rotation center to the actual robot center.
        # Motor status will be used for calculating raw odometry of the robot
        # and will be merged by Robot Localization node to get stabilized
        # odometry. Odom adapter will convert the raw odometry to the odometry
        # of the robot rotating center which is controlled by offset.
        #
        #                    (5~10Hz)                      /cabot/cmd_vel_adapter (passthrough)
        # +================+ /cmd_vel    +===============+                   +===============+
        # |                |============>| *             |==================>| *             |
        # | Controller     |             | OdomAdapter   |                   | SpeedControl  |
        # |                |<============|               |                   |               |
        # +================+ /odom       +===============+                   +===============+
        #                                            A                                    |
        #                                            | /cabot/odometry/filtered           | /cabot/cmd_vel  # noqa: E501
        #                   /cabot/imu/data          | (100Hz)                            | (target_rate: 40hz)
        # +================+             +===================+                            |
        # |*Cabot Sensor   |============>| *                 |   (passthrough)            |
        # |================|             | RobotLocalization |  /cabot/odom_raw           |
        #                                |                   |<==================+        |
        #                                +===================+                   |        |
        #                                                                        |        |
        #                                                     (passthrough)      |        |
        #                           (40Hz)                   /cabot/motorStatus  |        v
        # +================+             +==================+               +===============+
        # |                |============>|                  |==============>| *             |
        # | Motor          |   Serial    | MotorControl     |               | MotorAdapter  |
        # |                |<============|                  |<==============|               |
        # +================+             +==================+               +===============+
        #                           (40Hz)                   /cabot/motorTarget (target_rate: 40Hz)

        Node(
            package='cabot',
            executable='odom_adapter_node',
            namespace='/cabot',
            name='odom_adapter_node',
            output=output,
            parameters=[
                *param_files,
                {
                    'use_sim_time': use_sim_time,
                    'max_speed': max_speed
                },
            ],
        ),
        # for local odom navigation
        Node(
            package='cabot',
            executable='odom_adapter_node',
            namespace='/cabot',
            name='odom_adapter_node2',
            output=output,
            parameters=[*param_files, {'use_sim_time': use_sim_time}],
        ),
        # Cabot Lidar Speed Control
        Node(
            package='cabot',
            executable='lidar_speed_control_node',
            namespace='/cabot',
            name='lidar_speed_control_node',
            output=output,
            parameters=[*param_files, {'use_sim_time': use_sim_time}],
        ),
        # Cabot People Speed Control
        Node(
            package='cabot',
            executable='people_speed_control_node',
            namespace='/cabot',
            name='people_speed_control_node',
            output=output,
            parameters=[*param_files, {'use_sim_time': use_sim_time}],
        ),
        # Cabot TF Speed Control
        Node(
            package='cabot',
            executable='tf_speed_control_node',
            namespace='/cabot',
            name='tf_speed_control_node',
            output=output,
            parameters=[*param_files, {'use_sim_time': use_sim_time}],
        ),
        # Cabot Touch Speed Control
        Node(
            package='cabot',
            executable='touch_speed_control_node',
            namespace='/cabot',
            name='touch_speed_control_node',
            output=output,
            parameters=[*param_files, {'use_sim_time': use_sim_time}],
        ),

        # Cabot Speed Control
        # This node limit the speed from the move_base based on specified topics
        #   /cabot/lidar_speed           - control by lidar sensor
        #   /cabot/map_speed             - control by map speed poi
        #   /cabot/people_speed          - control by surrounding people
        #   /cabot/queue_speed           - control by queue
        #   /cabot/tf_speed              - control by existence of specific tf
        #   /cabot/touch_speed_switched  - control by touch sensor
        #                  TODO use touch_enabled argument
        #   /cabot/user_speed            - control by user
        Node(
            package='cabot',
            executable='speed_control_node',
            namespace='/cabot',
            name=PythonExpression(['"speed_control_node_touch_', touch_enabled, '"']),
            output=output,
            parameters=[*param_files, {'use_sim_time': use_sim_time}],
        ),

        # Sensor fusion for stabilizing odometry
        Node(
            package='robot_localization',
            executable='ekf_node',
            namespace='/cabot',
            name='ekf_node',
            output=output,
            parameters=[*param_files, {'use_sim_time': use_sim_time}],
        ),
    ])
