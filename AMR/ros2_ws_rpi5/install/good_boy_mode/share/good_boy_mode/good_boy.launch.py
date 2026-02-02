from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_publisher_node = Node(
        package='good_boy_mode',
        executable='cam_tilt_pan',
        name='cam_tilt_pan',
        output='screen'
    )
    controller_to_serial_node = Node(
        package='good_boy_mode',
        executable='mobile_bot',
        name='mobile_bot',
        output='screen'
    )
    return LaunchDescription([
        camera_publisher_node,
        controller_to_serial_node
    ])