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
        # robot_state_publisher: procesa el URDF y genera el árbol cinemático
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # Puente exclusivo: Recibe tus comandos y es el único dueño de /joint_states
        Node(
            package='brazo4',
            executable='esp32_to_urdf_bridge.py',
            name='esp32_to_urdf_bridge',
            output='screen',
        ),
        # RViz2 para visualizar el entorno gráfico
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])