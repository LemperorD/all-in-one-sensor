# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
主启动文件 - 统一入口
用于选择仿真或现实环境，启动相应的系统
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup_launch_description(context, *args, **kwargs):
    """
    根据 use_sim 参数动态选择启动配置
    """
    bringup_dir = get_package_share_directory("bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    
    use_sim = context.launch_configurations.get('use_sim', 'false').lower() == 'true'
    
    if use_sim:
        # 仿真环境
        launch_file = "rm_navigation_simulation_launch.py"
        config_path = "simulation"
    else:
        # 现实环境
        launch_file = "rm_navigation_reality_launch.py"
        config_path = "reality"
    
    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    launch_dir,
                    "simulation" if use_sim else "reality",
                    launch_file
                )
            ),
            launch_arguments={
                "namespace": LaunchConfiguration("namespace"),
                "slam": LaunchConfiguration("slam"),
                "world": LaunchConfiguration("world"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "autostart": LaunchConfiguration("autostart"),
            }.items(),
        ),
    ])
    
    return [ld]


def generate_launch_description():
    """
    生成启动描述
    """
    
    declare_use_sim_cmd = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="是否使用仿真环境 (Gazebo)。true=仿真, false=现实",
    )
    
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="机器人命名空间",
    )
    
    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="false",
        description="是否启用SLAM。true=启用, false=禁用",
    )
    
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmuc_2025",
        description="选择仿真场景 (如果use_sim=true)",
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否启用RViz可视化",
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="是否自动启动导航栈",
    )
    
    # 创建启动描述
    ld = LaunchDescription([
        declare_use_sim_cmd,
        declare_namespace_cmd,
        declare_slam_cmd,
        declare_world_cmd,
        declare_use_rviz_cmd,
        declare_autostart_cmd,
        OpaqueFunction(function=setup_launch_description),
    ])
    
    return ld
