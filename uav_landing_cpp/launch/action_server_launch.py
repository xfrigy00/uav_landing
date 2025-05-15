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

    topic_name_land_arg = DeclareLaunchArgument(
        'topic_name_land', 
        #default_value=TextSubstitution(text='/x500_1/aircraft/land'),
        default_value=TextSubstitution(text='/m100_1/aircraft/land'),
        description='Topic name for action land.'
    )

    return LaunchDescription([
        topic_name_cmd_vel_arg,
        topic_name_land_arg,
        Node(
            package='uav_landing_cpp',
            executable='action_server',
            name='action_server_node',
            parameters=[{
                'topic_name_cmd_vel': LaunchConfiguration('topic_name_cmd_vel'),
                'topic_name_land': LaunchConfiguration('topic_name_land'),
            }]
        ),
    ])
