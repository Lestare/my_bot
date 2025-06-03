import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Пути к пакетам
    pkg_rover = get_package_share_directory('my_bot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Аргументы запуска
    world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_rover, 'worlds', 'aruco_world.world'),
        description='Путь к миру Gazebo'
    )
    
    paused = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Запуск Gazebo в режиме паузы'
    )
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Использовать симулированное время'
    )
    
    gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Запуск GUI Gazebo'
    )
    
    headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Без GUI (headless режим)'
    )
    
    debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Режим отладки Gazebo'
    )
    
    # Запуск Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'pause': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui'),
            'verbose': LaunchConfiguration('debug'),
            'headless': LaunchConfiguration('headless'),
        }.items()
    )
    
    # Загрузка модели ровера
    robot_description_content = Command([
        'xacro ', 
        PathJoinSubstitution([pkg_rover, 'description', 'robot_core.xacro'])
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Публикатор описания робота
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content
        }]
    )
    
    # Спавн модели в Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        world,
        paused,
        use_sim_time,
        gui,
        headless,
        debug,
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])