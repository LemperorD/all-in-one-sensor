import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    trajectory_type_arg = DeclareLaunchArgument(
        'trajectory_type',
        default_value='circular',
        description='Trajectory type for target: circular, figure_8, spiral_up'
    )

    trajectory_radius_arg = DeclareLaunchArgument(
        'trajectory_radius',
        default_value='5.0',
        description='Radius of trajectory in meters'
    )

    trajectory_height_arg = DeclareLaunchArgument(
        'trajectory_height',
        default_value='2.0',
        description='Height of target trajectory in meters'
    )

    # Get package share directories
    simulator_share = FindPackageShare('simulator')

    # Set Gazebo model path
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(
            FindPackageShare('simulator').find('simulator'),
            'models'
        )
    )

    # Start Gazebo
    gazebo = Node(
        package='gazebo_ros',
        executable='gazebo',
        arguments=[
            os.path.join(
                FindPackageShare('simulator').find('simulator'),
                'worlds',
                'gimbal_sim.world'
            ),
            '--verbose'
        ],
        output='screen'
    )

    # Spawn gimbal platform
    spawn_gimbal = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'gimbal_platform',
            '-file', os.path.join(
                FindPackageShare('simulator').find('simulator'),
                'urdf',
                'gimbal_platform.urdf'
            ),
            '-x', '0',
            '-y', '0',
            '-z', '0'
        ],
        output='screen'
    )

    # Spawn QR target
    spawn_target = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'qr_code_target',
            '-file', os.path.join(
                FindPackageShare('simulator').find('simulator'),
                'models/qr_code_target',
                'model.sdf'
            ),
            '-x', '5',
            '-y', '0',
            '-z', '2'
        ],
        output='screen'
    )

    # Gimbal controller node
    gimbal_controller = Node(
        package='simulator',
        executable='gimbal_controller_node',
        output='screen',
        parameters=[
            {'max_pan_rate': 2.0},
            {'max_tilt_rate': 2.0},
            {'control_period': 0.01}
        ]
    )

    # Target tracker node
    target_tracker = Node(
        package='simulator',
        executable='target_tracker_node',
        output='screen',
        parameters=[
            {'trajectory_type': LaunchConfiguration('trajectory_type')},
            {'trajectory_radius': LaunchConfiguration('trajectory_radius')},
            {'trajectory_height': LaunchConfiguration('trajectory_height')},
            {'trajectory_period': 20.0},
            {'publish_rate': 10.0}
        ]
    )

    # Gazebo bridge node
    gazebo_bridge = Node(
        package='simulator',
        executable='gazebo_bridge_node',
        output='screen',
        parameters=[
            {'publish_tf': True},
            {'camera_frame_id': 'camera_optical_frame'},
            {'lidar_frame_id': 'lidar_link'}
        ]
    )

    return LaunchDescription([
        trajectory_type_arg,
        trajectory_radius_arg,
        trajectory_height_arg,
        set_model_path,
        gazebo,
        spawn_gimbal,
        spawn_target,
        gimbal_controller,
        target_tracker,
        gazebo_bridge,
    ])
