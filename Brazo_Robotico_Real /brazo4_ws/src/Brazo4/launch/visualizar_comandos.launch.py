import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('brazo4')
    urdf_file = os.path.join(pkg_share, 'urdf', 'brazo4central.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'brazo4.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # robot_state_publisher: publica las transformadas del URDF usando /joint_states
        # Se reasigna el canal de entrada a /joint_states_target para evitar conflictos con la simulación real
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings=[('/joint_states', '/joint_states_target')]
        ),
        # imprimir_comandos: lee los comandos target_deg y publica en /joint_states_target para RViz
        Node(
            package='brazo4',
            executable='imprimir_comandos.py',
            name='imprimir_comandos',
            output='screen',
        ),
        # RViz2 para visualizar
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
