from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Check if simulation
    is_sim = os.environ.get('CABOT_GAZEBO', '0') == '1'
    
    default_image_topic = '/rs1/color/image_raw'
    if is_sim:
        default_image_topic = '/camera/color/image_raw'

    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic', default_value=default_image_topic
    )
    view_image_arg = DeclareLaunchArgument(
        'view_image', default_value='True'
    )
    
    # YOLO Person Node
    yolo_person = Node(
        package='vsn_yolo_ros',
        executable='detect_person',
        name='yolo_person',
        parameters=[{
            'input_image_topic': LaunchConfiguration('input_image_topic'),
            'view_image': LaunchConfiguration('view_image'),
        }],
        output='screen'
    )

    # YOLO Gesture Node
    yolo_gesture = Node(
        package='vsn_yolo_ros',
        executable='detect_gesture',
        name='yolo_gesture',
        parameters=[{
            'input_image_topic': LaunchConfiguration('input_image_topic'),
            'view_image': False, # Usually don't pop up multiple windows
        }],
        output='screen'
    )

    # Vision2Social Node
    vision2social = Node(
        package='vlm_social_nav',
        executable='vision2social.py',
        name='vision2social',
        parameters=[{
            'input_image_topic': LaunchConfiguration('input_image_topic'),
            'view_image': LaunchConfiguration('view_image'),
        }],
        output='screen'
    )

    return LaunchDescription([
        input_image_topic_arg,
        view_image_arg,
        yolo_person,
        yolo_gesture,
        vision2social
    ])
