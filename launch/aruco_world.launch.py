import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessIO
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'my_bot'
    
    # ===== RSP (Robot State Publisher) =====
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'false'
        }.items()
    )

    # Путь к world-файлу с проверкой
    world_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'aruco_world.world'
    )
    
    if not os.path.exists(world_path):
        return LaunchDescription([
            LogInfo(msg=f"ERROR: World file not found: {world_path}")
        ])

    # Установка переменных окружения для Gazebo
    gazebo_env = os.environ.copy()
    gazebo_env['GAZEBO_MODEL_PATH'] = os.path.join(
        get_package_share_directory(package_name), 
        'models'
    ) + ':' + gazebo_env.get('GAZEBO_MODEL_PATH', '')
    
    gazebo_env['GAZEBO_RESOURCE_PATH'] = os.path.join(
        get_package_share_directory(package_name),
        'media'
    ) + ':' + gazebo_env.get('GAZEBO_RESOURCE_PATH', '')

    # Запуск Gazebo (упрощенная команда)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path],
        output='screen',
        env=gazebo_env
    )
    
    # Спавн робота
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-z', '0.1'],
        output='screen'
    )

    # Более надежный обработчик для спавна
    spawn_entity_handler = RegisterEventHandler(
        event_handler=OnProcessIO(
            target_action=gazebo,
            on_stdout=lambda event: spawn_entity,
        )
    )

    # Статическая трансформация для камеры
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )

    # Публикатор информации о камере
    camera_info_publisher = Node(
        package='my_bot',
        executable='camera_info_publisher.py',
        name='camera_info_publisher',
        output='screen',
        parameters=[{
            'camera_name': 'camera',
            'image_width': 640,
            'image_height': 480,
            'camera_frame_id': 'camera_link'
        }]
    )

    # Детектор ArUco меток
    aruco_detector = Node(
        package='my_bot',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen',
        parameters=[
            {'marker_size': 0.15},
            {'dictionary': 'DICT_4X4_50'},
            {'camera_frame': 'camera_link'},
            {'image_topic': '/camera/image_raw'},
            {'camera_info_topic': '/camera/camera_info'}
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
        LogInfo(msg="Starting ArUco simulation..."),
        rsp,
        gazebo,
        spawn_entity_handler,
        static_tf_camera,
        camera_info_publisher,
        aruco_detector,
        rviz_node
    ])