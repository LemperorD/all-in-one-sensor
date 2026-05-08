from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    px4_dir = LaunchConfiguration('px4_dir')
    world_file = LaunchConfiguration('world_file')
    model_sdf = LaunchConfiguration('model_sdf')
    start_ign = LaunchConfiguration('start_ign')
    mavros_params = LaunchConfiguration('mavros_params')

    def _launch(context, *args, **kwargs):
        px4_dir_val = context.perform_substitution(px4_dir)
        world_val = context.perform_substitution(world_file)
        model_val = context.perform_substitution(model_sdf)
        start_ign_val = context.perform_substitution(start_ign)
        mavros_params_val = context.perform_substitution(mavros_params)

        actions = []

        # Start PX4 SITL (uses make in PX4-Autopilot). Adjust target if needed.
        px4_cmd = ['bash', '-lc', f'cd "{px4_dir_val}" && make px4_sitl ignition >/dev/null 2>&1']
        actions.append(ExecuteProcess(cmd=px4_cmd, shell=False, output='screen'))

        # Start Ignition Gazebo with the chosen world (optional)
        if str(start_ign_val).lower() in ('true', '1', 'yes'):
            ign_cmd = ['bash', '-lc', f'ign gazebo "{world_val}" -r']
            actions.append(ExecuteProcess(cmd=ign_cmd, shell=False, output='screen'))

        # Spawn model into Ignition using ros_ign_gazebo create (requires ros_ign_gazebo)
        if model_val:
            # spawn_cmd = ['bash', '-lc', f'ros2 run ros_ign_gazebo create -file "{model_val}" -name iris -allow_renaming true']
            spawn_cmd = ['bash', '-lc', f'ros2 run ros_gz_sim create -file "{model_val}" -name iris -allow_renaming true']
            actions.append(ExecuteProcess(cmd=spawn_cmd, shell=False, output='screen'))

        # Start MAVROS. Use provided params file to connect to PX4 SITL via UDP.
        mavros_cmd = ['bash', '-lc', f'ros2 run mavros mavros_node --ros-args --params-file "{mavros_params_val}"']
        actions.append(ExecuteProcess(cmd=mavros_cmd, shell=False, output='screen'))

        return actions

    # compute package share paths for defaults
    pkg_share = get_package_share_directory('simulator')
    default_world = os.path.join(pkg_share, 'resource', 'worlds', 'gimbal_sim.sdf')
    default_model = os.path.join(pkg_share, 'resource', 'models', 'iris', 'model.sdf')
    default_mavros = os.path.join(pkg_share, 'config', 'mavros_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot'), description='PX4 source directory'),
        DeclareLaunchArgument('world_file', default_value=default_world, description='Ignition world file'),
        DeclareLaunchArgument('model_sdf', default_value=default_model, description='SDF model to spawn'),
        DeclareLaunchArgument('start_ign', default_value='True', description='Whether to start Ignition in this launch'),
        DeclareLaunchArgument('mavros_params', default_value=default_mavros, description='Path to MAVROS params file'),
        OpaqueFunction(function=_launch),
    ])
