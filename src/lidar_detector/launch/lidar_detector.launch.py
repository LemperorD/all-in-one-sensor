import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for lidar_detector node."""
    
    # Get the package directory
    pkg_dir = get_package_share_directory('lidar_detector')
    config_file = os.path.join(pkg_dir, 'config', 'lidar_detector.yaml')
    
    # Create a container for the component node
    container = ComposableNodeContainer(
        name='lidar_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_detector',
                plugin='lidar_detector::LidarDetectorNode',
                name='lidar_detector',
                parameters=[config_file],
                remappings=[
                    # Subscribe to point cloud from fast_lio
                    ('/cloud_registered', '/cloud_registered'),
                ],
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([
        container,
    ])
