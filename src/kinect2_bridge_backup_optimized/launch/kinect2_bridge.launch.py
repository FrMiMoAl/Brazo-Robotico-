from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Only the most commonly changed params are exposed as launch args.
    # All others can be overridden with: --ros-args -p <name>:=<value>
    pipeline_arg = DeclareLaunchArgument(
        'pipeline', default_value='opengl',
        description='Packet pipeline: opengl (default, uses GPU) or cpu')

    pub_resized_arg = DeclareLaunchArgument(
        'publish_resized_color', default_value='false',
        description='Publish a reduced-resolution color stream')

    kinect_node = Node(
        package='kinect2_bridge',
        executable='kinect2_bridge_node',
        name='kinect2_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'pipeline':              LaunchConfiguration('pipeline'),
            'publish_color':         True,
            'publish_depth':         True,
            'publish_ir':            True,
            'publish_resized_color': LaunchConfiguration('publish_resized_color'),
            'resized_width':         640,
            'resized_height':        360,
            'color_frame_id':        'kinect2_color_optical_frame',
            'depth_frame_id':        'kinect2_depth_optical_frame',
            'ir_frame_id':           'kinect2_ir_optical_frame',
            'timeout_ms':            1000,
        }],
    )

    return LaunchDescription([pipeline_arg, pub_resized_arg, kinect_node])
