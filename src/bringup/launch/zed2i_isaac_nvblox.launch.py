#!/usr/bin/env python3
"""
ZED2i Isaac ROS nvblox Integration Launch File

This launch file starts ZED2i camera integrated with NVIDIA Isaac ROS nvblox.
- ZED2i camera driver (Isaac ROS compatible configuration)
- nvblox 3D mapping
- ZED Gesture Controller (gesture control)
- Visualization tools (optional)

Usage:
  ros2 launch bringup zed2i_isaac_nvblox.launch.py
  ros2 launch bringup zed2i_isaac_nvblox.launch.py enable_visualization:=false
  ros2 launch bringup zed2i_isaac_nvblox.launch.py camera_name:=zed2i

Features:
  - Robot control with gestures (forward, turn, lateral movement)
  - Navigation goal setting with gestures
  - Mode switching with PS5 controller
"""

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    ZED2i + nvblox integrated launch file
    """

    # Define launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='zed',
        description='Name of the camera namespace'
    )

    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model (zed2i fixed for Isaac ROS)'
    )

    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable RViz2 visualization',
        choices=['true', 'false']
    )

    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='nvblox_container',
        description='Name of the component container for nvblox and ZED'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level',
        choices=['debug', 'info', 'warn', 'error']
    )

    # Get launch configurations
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    enable_visualization = LaunchConfiguration('enable_visualization')
    container_name = LaunchConfiguration('container_name')
    log_level = LaunchConfiguration('log_level')

    # Get package directories
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    nvblox_examples_dir = get_package_share_directory('nvblox_examples_bringup')

    # Create component container
    # Run nvblox and ZED camera in same container (for efficiency)
    container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        emulate_tty=True,
    )

    # Launch ZED2i camera driver (Isaac ROS compatible configuration)
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed2i_isaac_camera.launch.py')
        ),
        launch_arguments={
            'camera_name': camera_name,
            'camera_model': camera_model,
            'container_name': container_name,
            'publish_urdf': 'true',
            'publish_tf': 'true',
            'publish_map_tf': 'false',  # nvblox publishes map->odom
            'publish_imu_tf': 'false',
        }.items()
    )

    # Launch nvblox 3D mapping
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nvblox_examples_dir, 'launch', 'perception', 'nvblox.launch.py')
        ),
        launch_arguments={
            'container_name': container_name,
            'mode': 'static',  # Static environment mapping mode
            'camera': 'zed2',  # Use ZED2 settings internally in nvblox
        }.items()
    )

    # ZED Gesture Controller (gesture control)
    zed_gesture_controller_dir = get_package_share_directory('zed_gesture_controller')
    zed_gesture_controller_config = os.path.join(
        zed_gesture_controller_dir, 'config', 'zed_gesture_controller.yaml'
    )

    zed_gesture_controller_node = Node(
        package='zed_gesture_controller',
        executable='zed_gesture_controller',
        name='zed_gesture_controller',
        output='screen',
        parameters=[zed_gesture_controller_config],
        emulate_tty=True,
    )

    # Delayed launch of ZED Gesture Controller after camera startup
    load_zed_gesture_controller = TimerAction(
        period=5.0,  # Wait 5 seconds for ZED camera to fully start
        actions=[zed_gesture_controller_node]
    )

    # Visualization (optional)
    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nvblox_examples_dir, 'launch', 'visualization', 'visualization.launch.py')
        ),
        launch_arguments={
            'mode': 'static',
            'camera': 'zed2',
        }.items(),
        condition=IfCondition(enable_visualization)
    )

    # Log information
    log_info_start = LogInfo(
        msg='========================================\n'
            'Starting ZED2i Isaac ROS nvblox Integration\n'
            '========================================\n'
            'Camera: ZED2i\n'
            'Container: nvblox_container\n'
            'nvblox mode: static\n'
            'Gesture Controller: enabled\n'
            'Visualization: enabled (if enable_visualization:=true)\n'
            '========================================'
    )

    log_info_topics = LogInfo(
        msg='\nExpected Topics:\n'
            '  /zed/zed_node/rgb/image_rect_color\n'
            '  /zed/zed_node/depth/depth_registered\n'
            '  /zed/zed_node/pose\n'
            '  /nvblox_node/mesh\n'
            '  /nvblox_node/map_slice\n'
            '\nTF Frames:\n'
            '  map -> odom -> zed_camera_link -> base_link\n'
    )

    # Build LaunchDescription
    return LaunchDescription([
        # Declare arguments
        camera_name_arg,
        camera_model_arg,
        enable_visualization_arg,
        container_name_arg,
        log_level_arg,

        # Log information
        log_info_start,
        log_info_topics,

        # Component container
        container,

        # Launch ZED2i camera
        zed_camera_launch,

        # Launch nvblox mapping
        nvblox_launch,

        # Launch ZED Gesture Controller (delayed)
        load_zed_gesture_controller,

        # Visualization (conditional)
        visualization_launch,

        # Completion log
        LogInfo(
            msg='\n========================================\n'
                'ZED2i Isaac ROS nvblox launched successfully!\n'
                'ZED Gesture Controller: enabled\n'
                'Use RViz2 to visualize the 3D map.\n'
                'Use gestures to control the robot.\n'
                '========================================'
        ),
    ])
