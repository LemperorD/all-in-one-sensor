import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for the MPC gimbal planner."""

    pkg_dir = get_package_share_directory('mpc_gimbal_planner')
    config_file = os.path.join(pkg_dir, 'config', 'mpc_gimbal_planner.yaml')

    container = ComposableNodeContainer(
        name='mpc_gimbal_planner_container',
        namespace='',
        package='rclcpp_components',
        # use the multi-threaded container where available
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='mpc_gimbal_planner',
                plugin='mpc_gimbal_planner::PlannerNode',
                name='planner_node',
                parameters=[config_file],
                remappings=[],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        container,
    ])
