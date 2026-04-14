#!/usr/bin/env python3
"""
Data logging node for simulator - records trajectories and performance metrics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
from yolo_msgs.msg import DetectionArray
import json
import os
from datetime import datetime
import numpy as np

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        # Declare parameters
        self.declare_parameter('log_dir', '/tmp/gimbal_sim_logs')
        self.declare_parameter('record_trajectory', True)
        self.declare_parameter('record_gimbal_state', True)
        self.declare_parameter('record_commands', True)

        # Get parameters
        self.log_dir = self.get_parameter('log_dir').value
        self.record_trajectory = self.get_parameter('record_trajectory').value
        self.record_gimbal_state = self.get_parameter('record_gimbal_state').value
        self.record_commands = self.get_parameter('record_commands').value

        # Create log directory
        os.makedirs(self.log_dir, exist_ok=True)

        # Data storage
        self.data = {
            'metadata': {
                'start_time': datetime.now().isoformat(),
                'node_name': self.get_namespace()
            },
            'trajectory': [],
            'gimbal_state': [],
            'gimbal_commands': []
        }

        # Subscribers
        if self.record_trajectory:
            self.trajectory_sub = self.create_subscription(
                DetectionArray,
                'predicted_trajectory',
                self.trajectory_callback,
                qos_profile=QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                    reliability=ReliabilityPolicy.BEST_EFFORT
                )
            )

        if self.record_gimbal_state:
            self.gimbal_state_sub = self.create_subscription(
                Float32MultiArray,
                'gimbal_state',
                self.gimbal_state_callback,
                qos_profile=QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                    reliability=ReliabilityPolicy.BEST_EFFORT
                )
            )

        if self.record_commands:
            self.gimbal_cmd_sub = self.create_subscription(
                TwistStamped,
                'gimbal_command',
                self.gimbal_command_callback,
                qos_profile=QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                    reliability=ReliabilityPolicy.BEST_EFFORT
                )
            )

        # Timer to save data periodically
        self.save_timer = self.create_timer(5.0, self.save_data)

        self.get_logger().info(f"Data logger initialized. Log directory: {self.log_dir}")

    def trajectory_callback(self, msg):
        if not self.record_trajectory:
            return

        for detection in msg.detections:
            entry = {
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'position': {
                    'x': float(detection.bbox3d.center.position.x),
                    'y': float(detection.bbox3d.center.position.y),
                    'z': float(detection.bbox3d.center.position.z)
                },
                'size': {
                    'x': float(detection.bbox3d.size.x),
                    'y': float(detection.bbox3d.size.y),
                    'z': float(detection.bbox3d.size.z)
                },
                'score': float(detection.score)
            }
            self.data['trajectory'].append(entry)

    def gimbal_state_callback(self, msg):
        if not self.record_gimbal_state:
            return

        entry = {
            'timestamp': self.get_clock().now().nanoseconds * 1e-9,
            'pan': float(msg.data[0]) if len(msg.data) > 0 else 0.0,
            'tilt': float(msg.data[1]) if len(msg.data) > 1 else 0.0
        }
        self.data['gimbal_state'].append(entry)

    def gimbal_command_callback(self, msg):
        if not self.record_commands:
            return

        entry = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'pan': float(msg.twist.linear.x),
            'tilt': float(msg.twist.linear.y),
            'pan_rate': float(msg.twist.angular.z),
            'tilt_rate': float(msg.twist.angular.x)
        }
        self.data['gimbal_commands'].append(entry)

    def save_data(self):
        """Save data to file periodically"""
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self.log_dir, f'simulation_log_{timestamp}.json')

            # Update end time
            self.data['metadata']['end_time'] = datetime.now().isoformat()
            self.data['metadata']['num_trajectory_points'] = len(self.data['trajectory'])
            self.data['metadata']['num_gimbal_states'] = len(self.data['gimbal_state'])
            self.data['metadata']['num_commands'] = len(self.data['gimbal_commands'])

            with open(filename, 'w') as f:
                json.dump(self.data, f, indent=2)

            self.get_logger().debug(f"Data saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()  # Final save on shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
