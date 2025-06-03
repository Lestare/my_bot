import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Remote command relay
        Node(
            package='my_bot',
            executable='remote',
            name='remote'
        ),
        
        # Keyboard teleop
        Node(
            package='my_bot',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            output='screen',
            prefix='xterm -e'  # Открывает в отдельном терминале
        )
    ])