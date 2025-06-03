import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    package_name = 'my_bot'
    
    # Получаем путь к URDF
    pkg_share = get_package_share_directory(package_name)
    xacro_path = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    
    # Преобразуем XACRO в URDF
    robot_description = Command(['xacro ', xacro_path])
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),
        
        # Gazebo
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            arguments=['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so']
        ),
        
        # Spawn Entity
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_bot', '-topic', 'robot_description'],
            output='screen'
        ),
        
        # Relay Cmd
        Node(
            package='my_bot',
            executable='relay_cmd',
            name='relay_cmd'
        ),
        
        # TF Broadcaster
        Node(
            package='my_bot',
            executable='rover',
            name='tf_broadcaster'
        ),
        
        # Description Relay
        Node(
            package='my_bot',
            executable='description_relay',
            name='description_relay',
            parameters=[{'urdf_path': xacro_path}]
        )
    ])