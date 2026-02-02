from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_publisher_node = Node(
        package='human_control',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen'
    )
    controller_to_serial_node = Node(
        package='human_control',
        executable='controller_to_serial',
        name='controller_to_serial',
        output='screen'
    )
    return LaunchDescription([
        camera_publisher_node,
        controller_to_serial_node
    ])