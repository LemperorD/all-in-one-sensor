import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_simulator = get_package_share_directory("simulator")
    world_sdf_path = LaunchConfiguration("world_sdf_path")
    ign_config_path = LaunchConfiguration("ign_config_path")


    # Launch arguments
    trajectory_type_arg = DeclareLaunchArgument(
        "trajectory_type",
        default_value="circular",
        description="Trajectory type for target: circular, figure_8, spiral_up",
    )

    trajectory_radius_arg = DeclareLaunchArgument(
        "trajectory_radius",
        default_value="5.0",
        description="Radius of trajectory in meters",
    )

    trajectory_height_arg = DeclareLaunchArgument(
        "trajectory_height",
        default_value="2.0",
        description="Height of target trajectory in meters",
    )

    declare_world_sdf_path = DeclareLaunchArgument(
        "world_sdf_path",
        default_value=os.path.join(
            pkg_simulator, "worlds", "gimbal_sim.sdf"
        ),
        description="Path to the world SDF file",
    )

    declare_ign_config_path = DeclareLaunchArgument(
        "ign_config_path",
        default_value=os.path.join(pkg_simulator, "ign", "gui.config"),
        description="Path to the Ignition Gazebo GUI configuration file",
    )

    # Launch Gazebo Sim
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_version": "6",
            "gz_args": [
                world_sdf_path,
                TextSubstitution(text=" --gui-config "),
                ign_config_path,
            ],
        }.items(),
    )

    # Clock bridge
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # Gimbal controller node
    gimbal_controller = Node(
        package="simulator",
        executable="gimbal_controller_node",
        output="screen",
        parameters=[
            {"max_pan_rate": 2.0},
            {"max_tilt_rate": 2.0},
            {"control_period": 0.01},
        ],
    )

    # Target tracker node
    target_tracker = Node(
        package="simulator",
        executable="target_tracker_node",
        output="screen",
        parameters=[
            {"trajectory_type": LaunchConfiguration("trajectory_type")},
            {"trajectory_radius": LaunchConfiguration("trajectory_radius")},
            {"trajectory_height": LaunchConfiguration("trajectory_height")},
            {"trajectory_period": 20.0},
            {"publish_rate": 10.0},
        ],
    )

    # Gazebo bridge node
    gazebo_bridge = Node(
        package="simulator",
        executable="gazebo_bridge_node",
        output="screen",
        parameters=[
            {"publish_tf": True},
            {"camera_frame_id": "camera_optical_frame"},
            {"lidar_frame_id": "lidar_link"},
        ],
    )

    return LaunchDescription(
        [
            trajectory_type_arg,
            trajectory_radius_arg,
            trajectory_height_arg,
            declare_world_sdf_path,
            declare_ign_config_path,
            gazebo_sim,
            clock_bridge,
            gimbal_controller,
            target_tracker,
            gazebo_bridge,
        ]
    )
