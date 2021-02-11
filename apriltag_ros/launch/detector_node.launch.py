#
# launches detector as node
#
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch import LaunchDescription


def generate_launch_description():
    """Launch detector node."""
    node_name_arg = LaunchArg('node_name', default_value='detector',
                              description='name of detector node')
    node = Node(package='apriltag_ros',
                executable='apriltag_detector_node',
                output='screen',
                name=LaunchConfig('node_name'),
#                prefix=['xterm -e gdb -ex run --args'],
                parameters=[{'tag_family': 0,
                             'detector': 0,
                             'black_border_width': 2,
                             'decimate': 0}],
                remappings=[('/image', '/cam_0/synced/image_raw')])
    return LaunchDescription([node_name_arg, node])
