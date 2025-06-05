import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'

    # Запуск robot_state_publisher
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
        ])
    )

    # Спавн робота в Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Запуск RViz2 с конфигурацией
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory(package_name),
            'rviz',
            'config.rviz'  # Убедитесь что файл конфига существует
        )]
    )

    # Запуск телеоперации (2 варианта)
    # Вариант 1: Стандартный teleop_twist_keyboard
    # teleop_keyboard = Node(
    #   package='teleop_twist_keyboard',
    #    executable='teleop_twist_keyboard',
    #    output='screen',
    #    prefix='xterm -e',  # Требует установленного xterm
    #    remappings=[('/cmd_vel', '/cmd_vel_teleop')]  # Переназначение топика при необходимости
    # )
    
    # Вариант 2: Кастомный скрипт (раскомментировать если нужно)
    teleop_custom = ExecuteProcess(
        cmd=[
            'python3', 
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'keyboard_teleop.py'  # Убедитесь что файл существует
            )
        ],
        output='screen',
        shell=True
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz_node,
        teleop_custom  # Или teleop_custom для кастомного скрипта
    ])