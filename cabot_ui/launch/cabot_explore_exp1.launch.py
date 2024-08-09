from launch import LaunchDescription
from launch_ros.actions import Node
from datetime import datetime
import os

def generate_launch_description():
    # Get the current date and time
    now = datetime.now()
    exp_name = f'exp_date_{now.strftime("%Y%m%d")}_time_{now.strftime("%H%M")}'

    return LaunchDescription([
        Node(
            package='cabot_ui',
            executable='exploration_image_explain.py',
            name='exploration_image_explain',
            output='screen',
            parameters=[
                {'no_explain': False},
                {'log_dir': exp_name},
                {'once': False},
                {'semantic_map': False},
                {'intersection_detection': False},
                {'surronding_explain': True},
                {'sim': False},
                {'speak': True}
            ]
        ),
        Node(
            package='cabot_ui',
            executable='exploration_chat_server.py',
            name='exploration_chat_server',
            output='screen',
            parameters=[
                {'log_dir': exp_name},
                {'use_openai': True}  # Set this to False if you don't want to use OpenAI
            ],
        )
    ])
