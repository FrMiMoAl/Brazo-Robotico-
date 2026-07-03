#!/usr/bin/env python3
"""visual_sync.launch.py
Launches the robot model, static TF publisher and RViz for visual calibration.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to this package share directory
    pkg_share = get_package_share_directory('brazo_ai')

    # URDF path (adjust if your URDF filename is different)
    urdf_path = os.path.join(pkg_share, 'urdf', 'brazo.urdf')
    if not os.path.isfile(urdf_path):
        # fall back to a generic placeholder if missing
        urdf_path = ''

    # Robot State Publisher (publishes TF tree from URDF)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}] if urdf_path else []
    )

    # Static transform publisher – placeholder values (user will replace after calibration)
    static_tf = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--frame-id', 'base_link',
            '--child-frame-id', 'kinect2_depth_optical_frame',
            '0', '0', '0', '0', '0', '0'
        ],
        name='static_tf',
        output='screen'
    )

    # Joint state publisher (optional – publishes fake joint states if no hardware)
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # RViz configuration (you can edit the .rviz file later)
    rviz_config = os.path.join(pkg_share, 'rviz', 'visual_sync.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.isfile(rviz_config) else [],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        rsp_node,
        static_tf,
        joint_state_pub,
        rviz_node,
    ])
