import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('lidar_detector'),
        'config'
    )

    sensor_fusion_node = Node(
        package='lidar_detector',
        executable='sensor_fusion_node',
        name='sensor_fusion_node',
        parameters=[os.path.join(config_dir, 'sensor_fusion.yaml')],
        remappings=[
            ('detections', '/tracking'),
        ],
        output='screen',
    )

    return LaunchDescription([
        sensor_fusion_node,
    ])
