import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'
    pkg_path = get_package_share_directory(package_name)
    
    # Основные компоненты
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ])
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # RViz2 с конфигурацией
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'config.rviz')],
        output='screen'
    )

    # GUI для управления сочленениями
    joint_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Команда для запуска управления в отдельном терминале
    teleop_command = f"cd {os.path.join(pkg_path, 'launch')} && python3 keyboard_teleop.py"
    
    # Создаем действие для запуска в новом терминале
    teleop_terminal = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', teleop_command],
        output='screen'
    )

    # Обработчик события: запускаем управление после инициализации RViz
    launch_teleop = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rviz_node,
            on_start=[teleop_terminal]
        )
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz_node,
        joint_gui,
        launch_teleop
    ])