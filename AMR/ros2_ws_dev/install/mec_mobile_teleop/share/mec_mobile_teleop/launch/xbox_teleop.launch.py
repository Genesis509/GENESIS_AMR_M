from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'autorepeat': True,
                     'deadzone': 0.05,
                     'autorepeat_rate': 20.0}]
    )

    xbox_teleop_node = Node(
        package='mec_mobile_teleop',
        executable='xbox_teleop',
        name='xbox_teleop',
        output='screen'
    )

    camera_subscriber = Node(
        package='mec_mobile_teleop',
        executable='camera_subscriber',
        name='camera_subscriber',
        output='screen',
        parameters=[{'topic_name': '/camera/image_raw/compressed',
                     'window_name': 'Robot Camera Feed',
                     'display_fps': True,
                     'save_frames': False,
                     'save_path': '/tmp/frames'}]
    )

    return LaunchDescription([
        joy_node,
        xbox_teleop_node,
        camera_subscriber
    ])