import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_path = FindPackageShare('simulator').find('simulator')

    world_path = os.path.join(pkg_path, 'worlds', 'gimbal_sim.world')

    # 启动 gz 仿真
    gz_sim = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[world_path],
        output='screen'
    )

    # spawn gimbal（建议改成 sdf！）
    spawn_gimbal = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'gimbal_platform',
            '-file', os.path.join(pkg_path, 'urdf', 'gimbal_platform.urdf'),
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    # spawn target
    spawn_target = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'qr_code_target',
            '-file', os.path.join(pkg_path, 'models/qr_code_target/model.sdf'),
            '-x', '5', '-y', '0', '-z', '2'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        spawn_gimbal,
        spawn_target
    ])