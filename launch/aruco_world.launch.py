import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    package_name = 'my_bot'

    # Путь к нашему world-файлу
    world_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'aruco_world.world'
    )

    # Include robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Запуск Gazebo с нашим миром
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    # Спавн робота
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Измененный узел для ArUco
    aruco_marker_publisher = Node(
    package='aruco_opencv',  # Или 'ros2_aruco' в зависимости от установки
    executable='aruco_node',
    name='aruco_node',
    parameters=[
        {'image_topic': '/camera/image_raw'},
        {'camera_info_topic': '/camera/camera_info'},
        {'marker_size': 0.15},  # Размер метки в метрах
        {'dictionary': 'DICT_4X4_50'},  # Используем 'dictionary' вместо 'marker_dict'
    ]
)

    # Запуск RViz
    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'aruco.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        aruco_marker_publisher,
        rviz_node
    ])