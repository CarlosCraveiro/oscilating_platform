import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Caminhos para os diretórios dos pacotes
    crazyflie_dir = get_package_share_directory('crazyflie')
    my_pkg_dir = get_package_share_directory('oscillating_platform')

    # 2. Iniciar o Crazyswarm2 (com backend:=cflib)
    crazyflie_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(crazyflie_dir, 'launch', 'launch.py')
        ),
        launch_arguments={'backend': 'cflib'}.items()
    )

    # 3. O seu nó Estimador (O código que fizemos)
    estimator_node = Node(
        package='oscillating_platform',
        executable='ar_platform_estimator', # Nome configurado no setup.py
        name='platform_ar_estimator',
        output='screen'
    )

    # 4. Nó do Joystick que define nomes e modos de operacao
    joystick_node = Node(
        package='oscillating_platform',
        executable='joystick', # Substitua pelo nome real
        name='drone_joystick',
        output='screen'
    )

    # 5. RViz2 com o seu arquivo de configuração
    rviz_config_file = os.path.join(my_pkg_dir, 'rviz', 'config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        crazyflie_launch,
        estimator_node,
        joystick_node,
        rviz_node
    ])
