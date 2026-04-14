#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    MPC View Planner Node Launch Configuration

    Launches the MPC-based gimbal view planning node
    """

    mpc_view_planner_node = Node(
        package='mpc_gimbal_planner',
        executable='mpc_view_planner_executable',
        name='mpc_view_planner_node',
        output='screen',
        parameters=[
            # MPC parameters
            {'mpc_horizon': 10},
            {'mpc_dt': 0.1},
            {'planning_period': 0.1},

            # Cost function weights
            {'w_tracking': 1.0},
            {'w_smoothness': 0.5},
            {'w_control': 0.2},

            # Control constraints (rad/s, rad/s^2)
            {'max_pan_rate': 2.0},
            {'max_tilt_rate': 2.0},
            {'max_pan_accel': 1.0},
            {'max_tilt_accel': 1.0},

            # Camera parameters
            {'camera_fx': 1470.0},
            {'camera_fy': 1470.0},
            {'camera_cx': 480.0},
            {'camera_cy': 360.0},
        ],
        remappings=[
            ('predicted_trajectory', '/predicted_trajectory'),
            ('gimbal_state', '/gimbal_state'),
            ('gimbal_command', '/gimbal_command'),
        ]
    )

    return LaunchDescription([
        mpc_view_planner_node,
    ])
