#!/usr/bin/env python3
"""
ZED2i ZUPT Filter Launch File

Applies ZUPT (Zero-Velocity Update) filter to ZED2i odometry.

Features:
  - Detect stationary state from IMU data
  - Suppress Visual Odometry drift
  - Limit rapid velocity changes

Usage:
  ros2 launch zed_zupt_wrapper zed_zupt_filter.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package config directory path
    config_dir = os.path.join(
        get_package_share_directory('zed_zupt_wrapper'),
        'config'
    )

    # Parameter file path
    params_file = os.path.join(config_dir, 'zed_zupt_params.yaml')

    return LaunchDescription([
        Node(
            package='zed_zupt_wrapper',
            executable='zed_zupt_filter_node',
            name='zed_zupt_filter_node',
            output='screen',
            parameters=[params_file],
            emulate_tty=True,
        )
    ])
