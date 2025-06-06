import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'

    # Аргументы запуска
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='Gazebo world file name'
    )
    
    use_aruco_arg = DeclareLaunchArgument(
        'use_aruco',
        default_value='false',
        description='Enable ArUco localization'
    )

    # Пути
    world_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        LaunchConfiguration('world')
    )

    # Включение robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ]), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Запуск Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'), 
                'launch', 
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'true'
        }.items()
    )

    # Спавн робота
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Список действий по умолчанию
    launch_actions = [
        world_arg,
        use_aruco_arg,
        rsp,
        gazebo,
        spawn_entity
    ]

    # Условное добавление компонентов ArUco
    use_aruco = LaunchConfiguration('use_aruco')
    aruco_components = [
        # Статическая трансформация для камеры
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
            output='screen'
        ),
        
        # Публикатор информации о камере
        Node(
            package='my_bot',
            executable='camera_info_publisher.py',
            name='camera_info_publisher',
            output='screen'
        ),
        
        # Детектор ArUco меток
        Node(
            package='my_bot',
            executable='aruco_detector.py',
            name='aruco_detector',
            output='screen',
            parameters=[
                {'marker_size': 0.15},
                {'dictionary': 'DICT_4X4_50'},
            ]
        ),
        
        # Локализация по меткам
        Node(
            package='my_bot',
            executable='aruco_localization.py',
            name='aruco_localization',
            output='screen',
            parameters=[{
                'map_path': os.path.join(
                    get_package_share_directory(package_name),
                    'config',
                    'aruco_map.yaml'
                )
            }]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory(package_name),
                'config',
                'aruco.rviz'
            )],
            output='screen'
        )
    ]

    # Условное добавление компонентов ArUco
    for component in aruco_components:
        launch_actions.append(component)

    return LaunchDescription(launch_actions)