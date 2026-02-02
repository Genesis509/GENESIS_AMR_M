import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Declare launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev', 
        default_value='/dev/input/js0',
        description='Joystick device'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', 
        default_value='/cmd_vel',
        description='Command velocity topic name'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )

    # Include the built-in teleop_twist_joy launch file
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('teleop_twist_joy'),
                'launch',
                'teleop-launch.py'
            ])
        ]),
        launch_arguments={
            'joy_config': 'xbox',
            'joy_dev': LaunchConfiguration('joy_dev'),
            'config_filepath': '',
            'publish_stamped_twist': 'false'
        }.items()
    )

    # Optional: Remap cmd_vel topic if needed
    cmd_vel_remapper = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', LaunchConfiguration('cmd_vel_topic')],
        condition=IfCondition('false'),  # Set to 'true' if you need topic remapping
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        joy_dev_arg,
        cmd_vel_topic_arg,
        use_sim_time_arg,
        teleop_launch,
        # cmd_vel_remapper,  # Uncomment if needed
    ])