import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for immkf_predictor node."""

    pkg_dir = get_package_share_directory('immkf_predictor')
    config_file = os.path.join(pkg_dir, 'config', 'immkf_predictor.yaml')

    container = ComposableNodeContainer(
        name='immkf_predictor_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='immkf_predictor',
                plugin='immkf_predictor::PredictorNode',
                name='predictor_node',
                parameters=[config_file],
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([
        container,
    ])
