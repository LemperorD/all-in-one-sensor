from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os


def generate_launch_description():
    px4_dir = LaunchConfiguration('px4_dir')
    world_file = LaunchConfiguration('world_file')
    model_sdf = LaunchConfiguration('model_sdf')

    def _launch(context, *args, **kwargs):
        px4_dir_val = context.perform_substitution(px4_dir)
        world_val = context.perform_substitution(world_file)
        model_val = context.perform_substitution(model_sdf)

        actions = []

        # Start PX4 SITL (uses make in PX4-Autopilot). Adjust target if needed.
        px4_cmd = ['bash', '-lc', f'cd "{px4_dir_val}" && make px4_sitl_default >/dev/null 2>&1']
        actions.append(ExecuteProcess(cmd=px4_cmd, shell=False, output='screen'))

        # Start Ignition Gazebo with the chosen world
        ign_cmd = ['bash', '-lc', f'ign gazebo "{world_val}" -r']
        actions.append(ExecuteProcess(cmd=ign_cmd, shell=False, output='screen'))

        # Spawn model into Ignition using ros_ign_gazebo create (requires ros_ign_gazebo)
        if model_val:
            spawn_cmd = ['bash', '-lc', f'ros2 run ros_ign_gazebo create -file "{model_val}" -name iris -allow_renaming true']
            actions.append(ExecuteProcess(cmd=spawn_cmd, shell=False, output='screen'))

        # Start px4_ros_com node if available (user may replace with mavros)
        # Note: package must be installed and in ROS2 workspace
        px4_ros_com_cmd = ['bash', '-lc', 'ros2 run px4_ros_com px4_ros_com_node']
        actions.append(ExecuteProcess(cmd=px4_ros_com_cmd, shell=False, output='screen'))

        return actions

    return LaunchDescription([
        DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot'), description='PX4 source directory'),
        DeclareLaunchArgument('world_file', default_value=os.path.join(ThisLaunchFileDir(), '..', 'resource', 'worlds', 'gimbal_sim.sdf'), description='Ignition world file'),
        DeclareLaunchArgument('model_sdf', default_value=os.path.join(ThisLaunchFileDir(), '..', 'resource', 'models', 'iris', 'model.sdf'), description='SDF model to spawn'),
        OpaqueFunction(function=_launch),
    ])
