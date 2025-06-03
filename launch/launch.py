# Замените существующий launch_sim.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'my_bot'
    
    # URDF processing
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'description',
        'robot.urdf.xacro'
    )
    
    # Nodes
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_file, 'r').read(),
                'use_sim_time': True
            }]
        ),
        
        # Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_bot', '-topic', 'robot_description'],
            output='screen'
        ),
        
        # Relay nodes
        Node(
            package='my_bot',
            executable='relay_cmd.py',
            name='relay_cmd'
        ),
        Node(
            package='my_bot',
            executable='rover.py',
            name='tf_broadcaster'
        ),
        Node(
            package='my_bot',
            executable='description_relay.py',
            name='description_relay'
        )
    ])