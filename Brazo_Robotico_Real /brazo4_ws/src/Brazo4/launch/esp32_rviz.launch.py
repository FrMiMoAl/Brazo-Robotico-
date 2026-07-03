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
        # robot_state_publisher: publishes robot TF based on URDF and /joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # esp32_to_rviz: maps ESP32 topic values to RViz and publishes /joint_states
        Node(
            package='brazo4',
            executable='esp32_to_rviz.py',
            name='esp32_to_rviz',
            output='screen',
        ),
        # RViz2 to visualize the robot arm
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
