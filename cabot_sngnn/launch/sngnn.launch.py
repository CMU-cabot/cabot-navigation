from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cabot_sngnn',
            executable='sngnn_node',
            name='sngnn_node',
            output='screen',
            parameters=[
                {'model_path': '/home/developer/workspace/src/SNGNN2D-v2/model'}
            ]
        )
    ])
