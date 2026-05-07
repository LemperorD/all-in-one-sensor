import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_simulator = get_package_share_directory("simulator")

    world_sdf_path = LaunchConfiguration("world_sdf_path")
    ign_config_path = LaunchConfiguration("ign_config_path")

    declare_world_sdf_path = DeclareLaunchArgument(
        "world_sdf_path",
        default_value=os.path.join(
            pkg_simulator, "resource", "worlds", "gimbal_sim.sdf"
        ),
        description="Path to the gimbal sim world SDF file",
    )

    declare_ign_config_path = DeclareLaunchArgument(
        "ign_config_path",
        default_value=os.path.join(pkg_simulator, "resource", "ign", "gui.config"),
        description="Path to the Ignition Gazebo GUI configuration file",
    )

    # Allow choosing joystick device path
    declare_file_name = DeclareLaunchArgument(
        "file_name",
        default_value="/dev/input/js0",
        description="Joystick device file for rc_gimbal component",
    )

    # Composable node container and component for rc_gimbal
    rc_gimbal_component = ComposableNode(
        package="simulator",
        plugin="rc_gimbal::RcGimbalNode",
        name="rc_gimbal_node",
        parameters=[{"file_name": LaunchConfiguration("file_name")}],
    )

    container = ComposableNodeContainer(
        name="rc_gimbal_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[rc_gimbal_component],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_sdf_path)
    ld.add_action(declare_ign_config_path)
    ld.add_action(declare_file_name)
    ld.add_action(container)

    return ld
