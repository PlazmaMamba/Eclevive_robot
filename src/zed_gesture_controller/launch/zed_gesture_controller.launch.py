import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package config directory path
    config_dir = os.path.join(
        get_package_share_directory('zed_gesture_controller'),
        'config'
    )

    # YAML parameter file path
    params_file = os.path.join(config_dir, 'zed_gesture_controller.yaml')

    return LaunchDescription([
        Node(
            package='zed_gesture_controller',
            executable='zed_gesture_controller',
            name='zed_gesture_controller',
            output='screen',
            parameters=[params_file],
            emulate_tty=True,
        )
    ])
