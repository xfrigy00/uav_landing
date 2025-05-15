from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    topic_name_cmd_vel_arg = DeclareLaunchArgument(
        'topic_name_cmd_vel',
        #default_value=TextSubstitution(text='/x500_1/aircraft/cmd_vel'),
        default_value=TextSubstitution(text='/m100_1/aircraft/cmd_vel'),
        description='Topic name for cmd_vel'
    )

    return LaunchDescription([
        topic_name_cmd_vel_arg,
        Node(
            package='uav_landing_cpp',
            executable='action_client',
            name='action_client_node',
            parameters=[{
                'topic_name_cmd_vel': LaunchConfiguration('topic_name_cmd_vel'),
            }]
        ),
    ])
