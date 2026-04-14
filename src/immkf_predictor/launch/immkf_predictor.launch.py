from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'immkf_predictor'
    package_dir = get_package_share_directory(package_name)

    # Path to config file
    config_path = os.path.join(package_dir, 'config', 'immkf_default.yaml')

    # Create node
    immkf_node = Node(
        package=package_name,
        executable='immkf_predictor_node',
        name='immkf_predictor',
        parameters=[config_path],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        immkf_node,
    ])
