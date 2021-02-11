#
# launches detector as component
#
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg


def generate_launch_description():
    """Launch detector component."""
    container = ComposableNodeContainer(
            name='detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='apriltag_ros',
                    plugin='apriltag_ros::ApriltagDetectorComponent',
                    name=LaunchConfig('node_name'),
                    parameters=[{'tag_family': 0,
                                 'detector': 0,
                                 'black_border_width': 2,
                                 'decimate': 0}],
                    remappings=[('/image', '/cam_0/synced/image_raw'),
                                ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
    )
    name_arg = LaunchArg('node_name', default_value=['tag_detector'],
                         description='apriltag detector')
    return launch.LaunchDescription([name_arg, container])
