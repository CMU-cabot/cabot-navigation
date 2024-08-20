from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    log_dir = LaunchConfiguration('log_dir')
    apikey = LaunchConfiguration('apikey')

    return LaunchDescription([    
        # main code
        Node(
            package="cabot_ui",
            executable="exploration_main_loop.py",
            name="exploration_main_loop",
            output="screen",
            parameters=[
                {"log_dir": log_dir},
                {"dist_filter": True},
                {"is_sim": True},
            ],
        ),

        # surronding explain mode
        # Node(
        #     package='cabot_ui',
        #     executable='exploration_image_explain.py',
        #     name='exploration_image_explain_surronding',
        #     output='screen',
        #     parameters=[
        #         {'mode': 'surronding_explain_mode'},
        #         {'should_speak': True},
        #         {'log_dir': log_dir},                
        #         {'debug': False},
        #         {'once': False},
        #         {'no_explain_mode': False},
        #         {'is_sim': True},
        #         {'apikey': apikey}
                
        #     ]
        # ),

        # # chat server
        Node(
            package='cabot_ui',
            executable='exploration_chat_server.py',
            name='exploration_chat_server',
            output='screen',
            parameters=[
                {'log_dir': log_dir},
                {'use_openai': True},  # Set this to False if you don't want to use OpenAI
                {'apikey': apikey}
            ],
        ),

        # cancel node
        Node(
            package='cabot_ui',
            executable='exploration_cancel.py',
            name='exploration_cancel',
            output='screen',
        ),
    ])
