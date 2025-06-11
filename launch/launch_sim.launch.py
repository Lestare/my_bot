import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'
    
    # Используем ваш пользовательский world-файл
    world_path = os.path.join(
        get_package_share_directory(package_name), 
        'worlds',
        'empty.world'  # Убедитесь, что файл существует в этой папке
    )
    
    # 1. Robot State Publisher
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

    # 2. Настраиваем окружение Gazebo с учетом вашей системы
    gazebo_env = os.environ.copy()
    
    # Основные пути Gazebo
    gazebo_base_path = '/usr/share/gazebo-11'
    ros_gazebo_path = get_package_share_directory('gazebo_ros')
    custom_model_path = os.path.join(get_package_share_directory(package_name), 'models')
    custom_media_path = os.path.join(get_package_share_directory(package_name), 'media')
    
    # Формируем пути для Gazebo
    gazebo_env['GAZEBO_MODEL_PATH'] = ':'.join([
        gazebo_base_path + '/models',
        custom_model_path,
        os.environ.get('GAZEBO_MODEL_PATH', '')
    ])
    
    gazebo_env['GAZEBO_RESOURCE_PATH'] = ':'.join([
        gazebo_base_path,
        custom_media_path,
        ros_gazebo_path,
        os.environ.get('GAZEBO_RESOURCE_PATH', '')
    ])
    
    gazebo_env['GAZEBO_PLUGIN_PATH'] = ':'.join([
        '/opt/ros/foxy/lib',
        os.environ.get('GAZEBO_PLUGIN_PATH', '')
    ])

    # 3. Запуск Gazebo с правильными параметрами
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path  # Используем ваш пользовательский мир
        ],
        output='screen',
        env=gazebo_env
    )

    # 4. Спавн робота с задержкой
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-z', '0.1'],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        delayed_spawn,
    ])