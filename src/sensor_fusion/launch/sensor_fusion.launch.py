import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for sensor fusion node."""
    
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    config_file = os.path.join(pkg_dir, 'config', 'sensor_fusion.yaml')
    
    # Create a container for the component node
    container = ComposableNodeContainer(
        name='sensor_fusion_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='sensor_fusion',
                plugin='sensor_fusion::FusionNode',
                name='sensor_fusion_node',
                parameters=[config_file],
                remappings=[
                    # Remappings can be added here for subscribing to detections
                    # ('/dets2d', '/detections_2d'),
                    # ('/dets3d', '/detections_3d'),
                ],
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([
        container,
    ])
