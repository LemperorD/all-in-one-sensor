import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


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

	robot_xmacro_path = os.path.join(
		pkg_simulator,
		"resource",
		"xmacro",
		"all_in_one_sensor.xmacro",
	)

	bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
	aft_replace_ros_bridge_params = ReplaceString(
		source_file=bridge_config,
		replacements={"<robot_name>": "all_in_one_sensor"},
	)
	robot_config = os.path.join(pkg_simulator, "config", "base_params.yaml")

	xmacro = XMLMacro4sdf()
	xmacro.set_xml_file(robot_xmacro_path)
	xmacro.generate()
	robot_xml = xmacro.to_string()
	robot_spawn_xml = robot_xml.replace(
		"<parent>ground_plane</parent>",
		"<parent>world</parent>",
	)

	urdf_generator = UrdfGenerator()
	urdf_generator.parse_from_sdf_string(robot_xml)
	robot_urdf_xml = urdf_generator.to_string()
	if "<link name=\"ground_plane\"" not in robot_urdf_xml:
		robot_urdf_xml = robot_urdf_xml.replace(
			'<joint name="world_to_chassis" type="fixed">',
			'<link name="ground_plane" />\n\n\t<joint name="world_to_chassis" type="fixed">',
			1,
		)

	spawn_robot = Node(
		package="ros_gz_sim",
		executable="create",
		arguments=[
			"-string",
			robot_spawn_xml,
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

	robot_base = Node(
		package="simulator",
		executable="gz_gimbal",
		namespace="all_in_one_sensor",
		parameters=[robot_config, {"robot_name": "all_in_one_sensor"}],
	)

	robot_state_publisher = Node(
		package="robot_state_publisher",
		executable="robot_state_publisher",
		remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
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
		parameters=[{"config_file": aft_replace_ros_bridge_params}],
	)

	# Spawn iris model and trajectory node
	default_model = os.path.join(pkg_simulator, "resource", "models", "iris", "model.sdf")

	spawn_iris = Node(
		package="ros_gz_sim",
		executable="create",
		arguments=[
			"-file",
			default_model,
			"-name",
			"iris",
			"-allow_renaming",
			"true",
		],
	)

	trajectory_node = Node(
		package="simulator",
		executable="trajectory_node",
		output="screen",
		parameters=[{"model_name": "iris", "world_name": "gimbal_sim_world"}],
	)

	robot_ign_clock_bridge = Node(
		package="ros_gz_bridge",
		executable="parameter_bridge",
		arguments=[
			"/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
		],
	)

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
			"--req",
			'data: "all_in_one_sensor"',
		],
		output="screen",
	)

	ld = LaunchDescription()
	ld.add_action(declare_world_sdf_path)
	ld.add_action(declare_ign_config_path)
	ld.add_action(
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(
					get_package_share_directory("ros_gz_sim"),
					"launch",
					"gz_sim.launch.py",
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
	)
	ld.add_action(robot_ign_clock_bridge)
	ld.add_action(spawn_robot)
	ld.add_action(robot_base)
	ld.add_action(robot_state_publisher)
	ld.add_action(robot_ign_bridge)
	ld.add_action(spawn_iris)
	ld.add_action(trajectory_node)
	ld.add_action(set_performer_service)

	return ld

