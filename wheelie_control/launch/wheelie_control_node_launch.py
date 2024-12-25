import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pitch_threshold',
            default_value='-0.2',
            description='The pitch threshold for detecting wheelie state'
        ),
        
        Node(
            package='wheelie_control',  
            executable='wheelie_control_node',  
            name='wheelie_control_node',
            output='screen',
            parameters=[{'pitch_threshold': LaunchConfiguration('pitch_threshold')}],
        ),
    ])
