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
                executable='apriltag_pose_estimator',
                output='screen',
                name=LaunchConfig('node_name'),
#                prefix=['xterm -e gdb -ex run --args'],
                parameters=[{#'tag_family': 0,
                             'broadcast_tf': True,
                             'enable_all_tags': True,
                             'default_tag_size': 0.03,
                             'tag_descriptions.ids': [0, 100],
                             'tag_descriptions.sizes': [0.03, 0.163513],
                             'tag_descriptions.frame_ids': ["my_tag",""]
                            } ],
                remappings=[('/camera_info', '/camera0/camera_info'),
                            ('/apriltags', '/tags')])
    return LaunchDescription([node_name_arg, node])
