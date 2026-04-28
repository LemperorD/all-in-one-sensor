import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    pkg_simulator = get_package_share_directory("simulator")

    robot_xmacro_path = os.path.join(
        pkg_simulator,
        "resource",
        "xmacro",
        "all_in_one_sensor.xmacro",
    )
    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)

    ld = LaunchDescription()

    # Generate SDF from xmacro
    xmacro.generate()
    robot_xml = xmacro.to_string()

    # Generate URDF from SDF
    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_string(robot_xml)
    robot_urdf_xml = urdf_generator.to_string()

    # # replace the <robot_name> in the bridge config file
    # aft_replace_ros_bridge_params = ReplaceString(
    #     source_file=bridge_config,
    #     replacements={"<robot_name>": robot["name"]},
    # )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string",
            robot_xml,
            "-name",
            "all_in_one_sensor",
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
            "-Y",
            "0.0",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        remappings=remappings,
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_urdf_xml,
            }
        ],
    )

    robot_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
    )

    # Execute service call after spawning robots
    # https://gazebosim.org/api/gazebo/6.9/levels.html#Runtime-performers
    set_performer_service = ExecuteProcess(
        cmd=[
            "ign",
            "service",
            "-s",
            "/world/default/level/set_performer",
            "--reqtype",
            "ignition.msgs.StringMsg",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "2000",
            # "--req",
            # f'data: "{robot["name"]}"',
        ],
        output="screen",
    )

    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(robot_ign_bridge)
    ld.add_action(set_performer_service)

    return ld
