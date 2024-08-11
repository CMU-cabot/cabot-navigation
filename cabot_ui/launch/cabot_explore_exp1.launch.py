from launch import LaunchDescription
from launch_ros.actions import Node
from datetime import datetime
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get the current date and time
    now = datetime.now()
    exp_name = f'exp_date_{now.strftime("%Y%m%d")}_time_{now.strftime("%H%M")}'
    apikey = LaunchConfiguration('apikey')

    return LaunchDescription([
        # scemantic map mode
        # Node(
        #     package='cabot_ui',
        #     executable='exploration_image_explain.py',
        #     name='exploration_image_explain_scemantic',
        #     output='screen',
        #     parameters=[
        #         {'mode': 'semantic_map_mode'},
        #         {'should_speak': False},
        #         {'log_dir': exp_name},                
        #         {'debug': False},
        #         {'once': False},
        #         {'no_explain_mode': False},
        #         {'is_sim': True},
        #         {'apikey': apikey}
                
        #     ]
        # ),
        
        # surronding explain mode
        Node(
            package='cabot_ui',
            executable='exploration_image_explain.py',
            name='exploration_image_explain_surronding',
            output='screen',
            parameters=[
                {'mode': 'surronding_explain_mode'},
                {'should_speak': True},
                {'log_dir': exp_name},                
                {'debug': False},
                {'once': False},
                {'no_explain_mode': False},
                {'is_sim': True},
                {'apikey': apikey}
                
            ]
        ),

        # chat server
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
