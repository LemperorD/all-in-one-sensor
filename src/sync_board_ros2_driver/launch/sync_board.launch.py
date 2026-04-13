from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('sync_board_ros2_driver')
    config_file = os.path.join(pkg_share, 'config', 'sync_board_params.yaml')

    device_path_arg = DeclareLaunchArgument(
        'device_path', default_value='/dev/ttyACM0', description='Serial device path')
    baudrate_arg = DeclareLaunchArgument(
        'baudrate', default_value='921600', description='Serial baudrate')

    sync_node = Node(
        package='sync_board_ros2_driver',
        executable='sync_board_node',
        name='sync_board_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        device_path_arg,
        baudrate_arg,
        sync_node
    ])
