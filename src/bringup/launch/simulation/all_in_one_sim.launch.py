import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="all_in_one_sensor",
        description="Top-level namespace",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="True",
        description="Use simulation (Gazebo) clock if True",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file", default_value=os.path.join(
            bringup_dir, "config", "simulation", "all_in_one_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file", default_value=os.path.join(bringup_dir, "rviz", "all_in_one.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    # ==================== Perception Nodes ====================
    # Point Cloud Processing
    start_velodyne_convert_tool = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="ign_sim_pointcloud_tool",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    # SLAM (Fast LIO)
    start_fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fast_lio",
        output="screen",
        parameters=[configured_params],
    )

    # 2D Object Detection (YOLO)
    start_yolo_detector_node = Node(
        package="yolo_ros",
        executable="yolo_node",
        name="yolo_detector",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    # 3D Object Detection (LiDAR-based)
    start_lidar_detector_node = Node(
        package="lidar_detector",
        executable="lidar_detector_component",
        name="lidar_detector",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    # ==================== Fusion Node ====================
    # Multi-Sensor Fusion (2D + 3D Detection)
    start_fusion_node = Node(
        package="sensor_fusion",
        executable="fusion_node",
        name="fusion_node",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    # ==================== Tracking Node ====================
    # IMMKF Trajectory Predictor
    start_immkf_predictor_node = Node(
        package="immkf_predictor",
        executable="immkf_predictor_node",
        name="immkf_predictor",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    # ==================== Control Node ====================
    # MPC Gimbal Planner Controller
    start_mpc_planner_node = Node(
        package="mpc_gimbal_planner",
        executable="mpc_planner_node",
        name="mpc_gimbal_planner",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    # ==================== Visualization ====================
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add Perception Pipeline
    ld.add_action(start_velodyne_convert_tool)
    ld.add_action(start_fast_lio_node)
    ld.add_action(start_yolo_detector_node)
    ld.add_action(start_lidar_detector_node)

    # Add Fusion and Tracking Pipeline
    ld.add_action(start_fusion_node)
    ld.add_action(start_immkf_predictor_node)

    # Add Control Pipeline
    ld.add_action(start_mpc_planner_node)

    # Add Visualization
    ld.add_action(rviz_cmd)

    return ld
