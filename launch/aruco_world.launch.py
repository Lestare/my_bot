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
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Спавн робота
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Убедимся, что пути к медиа-ресурсам установлены правильно
    media_path = os.path.join(
        get_package_share_directory(package_name),
        'launch', 'models', 'aruco_wall', 'materials'
    )
    
    # Узел для публикации статических трансформаций камеры
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )

    # Узел для публикации статических трансформаций карты
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Узел для публикации информации о камере (заглушка)
    camera_info_publisher = Node(
        package='my_bot',
        executable='camera_info_publisher.py',
        name='camera_info_publisher',
        output='screen'
    )

    # Узел для визуализации маркеров в RViz
    aruco_marker_publisher = Node(
        package='my_bot',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen',
        parameters=[
            {'marker_size': 0.15},  # Размер метки в метрах
            {'dictionary': 'DICT_4X4_50'},
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
        static_tf_camera,
        static_tf_map,
        camera_info_publisher,
        aruco_marker_publisher,
        rviz_node
    ])