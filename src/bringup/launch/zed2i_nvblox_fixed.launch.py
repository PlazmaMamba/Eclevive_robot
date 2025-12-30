#!/usr/bin/env python3
"""
ZED2i Isaac ROS nvblox + All Devices Integration Launch File (Complete System)

This launch file starts the complete robot system:
- ZED2i camera (Visual SLAM, human skeleton detection)
- ZED ZUPT Filter (odometry drift prevention)
- NVIDIA Isaac ROS nvblox (3D mapping, extended range configuration)
- ZED Goal Publisher (gesture control)
- Front/Back LiDAR (scan merge, filtering)
- PS5 controller (teleoperation)
- micro-ROS Agent (microcontroller communication)
- Safety Sensor (safety monitoring)

Launched Nodes:
  1. Component Container & Robot State Publisher
  2. ZED2i camera node (Composable)
  3. Front/Back LiDAR + Laser Merger + Laser Filter
  4. micro-ROS Agent (mecanum wheel control)
  5. PS5 controller + Joy Mecanum Controller
  6. Safety Sensor
  7. Static TF Publishers (base_link, LiDAR placement)
  8. ZED ZUPT Filter (odometry stabilization)
  9. nvblox node (extended range 14m)
  10. ZED Goal Publisher node
  11. RViz2 (optional)

Usage:
  ros2 launch bringup zed2i_nvblox_fixed.launch.py

Options:
  ros2 launch bringup zed2i_nvblox_fixed.launch.py enable_visualization:=true
"""

import os
import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Launch setup with proper sequencing"""

    # Get launch configurations
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera_model = LaunchConfiguration('camera_model').perform(context)
    enable_visualization = LaunchConfiguration('enable_visualization').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)

    # Package directories
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    nvblox_examples_dir = get_package_share_directory('nvblox_examples_bringup')

    # Container name (no namespace prefix)
    container_name = 'nvblox_container'

    # ZED namespace
    zed_namespace = camera_name

    actions = []

    # ========================================
    # 1. Component Container
    # ========================================
    container = ComposableNodeContainer(
        name=container_name,
        namespace='',  # No namespace for container
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        emulate_tty=True,
    )
    actions.append(container)

    # ========================================
    # 2. Robot State Publisher (for URDF/TF)
    # ========================================
    xacro_path = os.path.join(zed_wrapper_dir, 'urdf', 'zed_descr.urdf.xacro')
    xacro_command = [
        'xacro', ' ', xacro_path, ' ',
        'camera_name:=', camera_name, ' ',
        'camera_model:=', camera_model
    ]

    rsp_node = Node(
        package='robot_state_publisher',
        namespace=zed_namespace,
        executable='robot_state_publisher',
        name=f'{camera_name}_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': Command(xacro_command)
        }]
    )
    actions.append(rsp_node)

    # ========================================
    # 3. ZED Camera Node (as composable node)
    # ========================================

    # ZED configuration files
    config_common = os.path.join(zed_wrapper_dir, 'config', 'common_stereo_isaac.yaml')
    config_camera = os.path.join(zed_wrapper_dir, 'config', f'{camera_model}.yaml')
    # config_ffmpeg = os.path.join(zed_wrapper_dir, 'config', 'ffmpeg.yaml')  # FFMPEG removed from ZED wrapper (Jetson incompatibility)
    config_od = os.path.join(zed_wrapper_dir, 'config', 'object_detection.yaml')
    config_custom_od = os.path.join(zed_wrapper_dir, 'config', 'custom_object_detection.yaml')

    zed_node_params = [
        config_common,
        config_camera,
        # config_ffmpeg,  # FFMPEG removed from ZED wrapper
        config_od,
        config_custom_od,
        {
            'use_sim_time': False,
            'general.camera_name': camera_name,
            'general.camera_model': camera_model,
            'pos_tracking.publish_tf': True,
            'pos_tracking.publish_map_tf': True,   # CRITICAL: Publish map->odom TF for nvblox compatibility
            # CRITICAL: DO NOT override map_frame and odometry_frame here!
            # These MUST use the values from common_stereo_isaac.yaml:
            #   map_frame: 'odom' (parent frame for Visual Odometry)
            #   odometry_frame: 'zed_camera_origin' (intermediate odometry frame)
            'sensors.publish_imu_tf': False,
        }
    ]

    zed_wrapper_component = ComposableNode(
        package='zed_components',
        namespace=zed_namespace,
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=zed_node_params,
        extra_arguments=[{'use_intra_process_comms': False}]  # CRITICAL: Must be False for NITROS compatibility
    )

    # Load ZED node into container with delay
    load_zed_node = TimerAction(
        period=2.0,  # Wait 2 seconds for container to be ready
        actions=[
            LoadComposableNodes(
                target_container=f'/{container_name}',
                composable_node_descriptions=[zed_wrapper_component]
            )
        ]
    )
    actions.append(load_zed_node)

    # ========================================
    # 4. Device Nodes (LiDAR, Controllers, etc.)
    # ========================================

    # 4-1. LiDAR Launch Files
    ldlidar_dir = get_package_share_directory('ldlidar_stl_ros2')

    # Back LiDAR
    back_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_dir, 'launch', 'back_stl27l.launch.py')
        )
    )
    actions.append(back_lidar_launch)

    # Front LiDAR
    front_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_dir, 'launch', 'stl27l.launch.py')
        )
    )
    actions.append(front_lidar_launch)

    # 4-2. Laser Merger
    laser_merger_dir = get_package_share_directory('laser_merger2')
    laser_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(laser_merger_dir, 'launch', 'laser_merger.launch.py')
        )
    )
    actions.append(laser_merger_launch)

    # 4-3. Laser Filter (Box filter for robot body)
    laser_filters_dir = get_package_share_directory('laser_filters')
    box_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(laser_filters_dir, 'examples', 'box_filter_example.launch.py')
        )
    )
    actions.append(box_filter_launch)

    # 4-4. micro-ROS Agent (Microcontroller communication)
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["multiserial", "--devs", "/dev/ttyACM1 /dev/ttyACM0 /dev/M5atom", "--baud", "1000000"],
        respawn=True,
        respawn_delay=5,
        output='screen'
    )
    actions.append(micro_ros_agent_node)

    # 4-5. Joy (PS5 Controller)
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'dev_name': 'DualSense Wireless Controller',
            'deadzone': 0.05,
            'autorepeat_rate': 30.0,
        }]
    )
    actions.append(joy_node)

    # 4-6. Joy Mecanum Controller
    joy_mecanum_node = Node(
        package='joy_mecanum_controller',
        executable='joy_mecanum_controller_node',
        name='joy_mecanum_controller',
        output='screen',
    )
    actions.append(joy_mecanum_node)

    # 4-7. Safety Sensor
    safety_sensor_node = Node(
        package='safety_sensor',
        executable='safety_sensor',
        name='safety_sensor',
        output='screen',
    )
    actions.append(safety_sensor_node)

    # 4-8. Static TF Publishers

    # map to odom (static TF, no drift correction)
    # nvblox publishes map in map frame, ZED2i publishes TF from odom
    # 2025-10-26 finalized version: map_frame uses 'odom' (ROS2 standard naming)
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    actions.append(tf_map_to_odom)

    # base_link to zed_camera_link
    tf_base_to_zed = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed2_static_transform',
        arguments=['-0.12', '0.0', '-0.18', '0.0', '0.0', '0.0', '3.14159', 'zed_camera_link', 'base_link'],
    )
    actions.append(tf_base_to_zed)

    # zed_camera_link to back_lidar
    tf_zed_to_back_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_camera_link_to_back_lidar',
        arguments=['-0.296', '0.0', '-0.03', '0.0', '3.14159', '0', 'zed_camera_link', 'back_lidar'],
    )
    actions.append(tf_zed_to_back_lidar)

    # zed_camera_link to front_lidar
    tf_zed_to_front_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_camera_link_to_stl27_laser',
        arguments=['-0.0', '0', '-0.03', '0', '0', '0', 'zed_camera_link', 'front_lidar'],
    )
    actions.append(tf_zed_to_front_lidar)

    # ========================================
    # 5. ZED ZUPT Filter (Odometry Stabilization)
    # ========================================

    # ZED ZUPT Filter configuration
    zed_zupt_dir = get_package_share_directory('zed_zupt_wrapper')
    zed_zupt_config = os.path.join(
        zed_zupt_dir, 'config', 'zed_zupt_params.yaml'
    )

    zed_zupt_node = Node(
        package='zed_zupt_wrapper',
        executable='zed_zupt_filter_node',
        name='zed_zupt_filter_node',
        output='screen',
        parameters=[zed_zupt_config],
        emulate_tty=True,
    )

    # Launch ZUPT filter after ZED camera is ready
    load_zed_zupt = TimerAction(
        period=3.0,  # Wait 3 seconds for ZED to be ready
        actions=[zed_zupt_node]
    )
    actions.append(load_zed_zupt)

    # ========================================
    # 6. nvblox Node
    # ========================================

    # nvblox configuration files
    nvblox_base_config = os.path.join(
        nvblox_examples_dir, 'config', 'nvblox', 'nvblox_base.yaml'
    )
    nvblox_zed_config = os.path.join(
        nvblox_examples_dir, 'config', 'nvblox', 'specializations', 'nvblox_zed.yaml'
    )

    # Extended range configuration (custom)
    bringup_dir = get_package_share_directory('bringup')
    nvblox_extended_config = os.path.join(
        bringup_dir, 'config', 'nvblox', 'nvblox_extended_range.yaml'
    )

    # Nav2 integration configuration (custom)
    nvblox_nav2_config = os.path.join(
        bringup_dir, 'config', 'nvblox', 'nvblox_nav2_integration.yaml'
    )

    # Topic remappings for ZED and Nav2
    nvblox_remappings = [
        ('camera_0/depth/image', f'/{zed_namespace}/zed_node/depth/depth_registered'),
        ('camera_0/depth/camera_info', f'/{zed_namespace}/zed_node/depth/camera_info'),
        ('camera_0/color/image', f'/{zed_namespace}/zed_node/rgb/image_rect_color'),
        ('camera_0/color/camera_info', f'/{zed_namespace}/zed_node/rgb/camera_info'),
        ('pose', f'/{zed_namespace}/zed_node/pose'),
        # Remap Occupancy Grid to /map (Nav2 compatible)
        # 2025-10-21 fix: static_occupancy_layer → static_occupancy_grid (correct topic name)
        ('static_occupancy_grid', '/map'),
        # LiDAR scan remapping (2025-10-21 addition: integrate filtered scan to nvblox)
        ('lidar_0/scan', '/scan_filtered'),
    ]

    nvblox_params = [
        nvblox_base_config,
        nvblox_zed_config,
        nvblox_extended_config,  # Extended range settings (overrides base settings)
        nvblox_nav2_config,      # Nav2 integration settings (map frame & occupancy grid)
        {
            'num_cameras': 1,
            'use_lidar': True,  # Enable LiDAR integration (2025-10-21: False→True)
            'global_frame': 'map',  # Nav2 expects map frame
        }
    ]

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=nvblox_remappings,
        parameters=nvblox_params,
    )

    # Load nvblox node into container with delay
    load_nvblox_node = TimerAction(
        period=4.0,  # Wait 4 seconds for ZED to be ready
        actions=[
            LoadComposableNodes(
                target_container=f'/{container_name}',
              
                composable_node_descriptions=[nvblox_node]
            )
        ]
    )
    actions.append(load_nvblox_node)

    # ========================================
    # 7. ZED Goal Publisher
    # ========================================

    # ZED Goal Publisher configuration
    zed_goal_publisher_dir = get_package_share_directory('zed_goal_publisher')
    zed_goal_publisher_config = os.path.join(
        zed_goal_publisher_dir, 'config', 'zed_goal_publisher.yaml'
    )

    zed_goal_publisher_node = Node(
        package='zed_goal_publisher',
        executable='zed_goal_publisher',
        name='zed_goal_publisher',
        output='screen',
        parameters=[zed_goal_publisher_config],
        emulate_tty=True,
    )

    # Launch ZED Goal Publisher after ZED camera is ready
    #load_zed_goal_publisher = TimerAction(
    #    period=5.0,  # Wait 5 seconds for ZED to be ready
    #    actions=[zed_goal_publisher_node]
    #)
    #actions.append(load_zed_goal_publisher)

    # ========================================
    # 8. Map→Odom TF Publisher (Disabled - ZED2i handles this)
    # ========================================

    # 2025-10-26 fix: map→zed_odom TF is published directly by ZED2i, this node is not needed
    # ZED2i settings: publish_map_tf: true, map_frame: 'map', odometry_frame: 'zed_odom'
    # TF tree: map -> zed_odom -> zed_camera_origin -> zed_camera_link
    #
    # map_odom_tf_publisher node has been removed

    # ========================================
    # 9. RViz2 (Optional)
    # ========================================

    if enable_visualization == 'true':
        rviz_config = os.path.join(
            nvblox_examples_dir, 'config', 'rviz', 'zed.rviz'
        )

        # Check if RViz config exists, otherwise use default
        if not os.path.exists(rviz_config):
            rviz_config = os.path.join(
                nvblox_examples_dir, 'config', 'rviz', 'default.rviz'
            )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            output='screen'
        )

        # Launch RViz after nodes are ready
        launch_rviz = TimerAction(
            period=6.0,
            actions=[rviz_node]
        )
        actions.append(launch_rviz)

    # ========================================
    # Log Messages
    # ========================================

    actions.append(LogInfo(msg=TextSubstitution(text=
        '\n========================================\n'
        'Complete Robot System Launch\n'
        '========================================\n'
        f'Camera: {camera_model}\n'
        f'Namespace: {zed_namespace}\n'
        f'Container: {container_name}\n'
        '\nEnabled Components:\n'
        '  ✓ ZED2i Camera (Visual SLAM + Body Tracking)\n'
        '  ✓ ZED ZUPT Filter (Odometry Stabilization)\n'
        '  ✓ nvblox (3D Mapping, Extended Range: 14m)\n'
        '  ✓ LiDAR Front/Back + Merger + Filter\n'
        '  ✓ PS5 Controller + Mecanum Control\n'
        '  ✓ micro-ROS Agent (Microcontroller)\n'
        '  ✓ ZED Goal Publisher (Gesture Control)\n'
        '  ✓ Safety Sensor\n'
        '========================================\n'
    )))

    actions.append(TimerAction(
        period=8.0,
        actions=[LogInfo(msg=TextSubstitution(text=
            '\n========================================\n'
            'Expected Topics:\n'
            '\n[ZED Camera]\n'
            f'  /{zed_namespace}/zed_node/rgb/image_rect_color\n'
            f'  /{zed_namespace}/zed_node/depth/depth_registered\n'
            f'  /{zed_namespace}/zed_node/pose\n'
            f'  /{zed_namespace}/zed_node/odom (Raw)\n'
            f'  /{zed_namespace}/zed_node/odom_zupt (ZUPT Filtered)\n'
            f'  /{zed_namespace}/zed_node/body_trk/skeletons\n'
            f'  /{zed_namespace}/zed_node/imu/data\n'
            '  /zupt/status (ZUPT Debug)\n'
            '\n[nvblox]\n'
            '  /nvblox_node/mesh\n'
            '  /nvblox_node/map_slice (14m range)\n'
            '  /nvblox_node/pointcloud\n'
            '\n[LiDAR]\n'
            '  /front_scan\n'
            '  /back_scan\n'
            '  /merged_scan\n'
            '  /merged_scan_filtered\n'
            '\n[Control]\n'
            '  /joy (PS5 Controller)\n'
            '  /cmd_vel (Mecanum)\n'
            '  /goal_pose (ZED Goal Publisher)\n'
            '\n[TF Tree]\n'
            '  map -> zed_odom -> zed_camera_origin -> zed_camera_link\n'
            '    ├─ base_link\n'
            '    ├─ front_lidar\n'
            '    └─ back_lidar\n'
            '\nVerification Commands:\n'
            '  ros2 node list\n'
            '  ros2 topic list\n'
            '  ros2 topic hz /merged_scan\n'
            '  ros2 topic hz /nvblox_node/map_slice\n'
            '  ros2 topic echo /cmd_vel\n'
            '  ros2 topic echo /zupt/status\n'
            f'  ros2 topic echo /{zed_namespace}/zed_node/odom_zupt\n'
            '  ros2 run tf2_tools view_frames\n'
            '  ros2 param get /nvblox_node map_clearing_radius_m\n'
            '  ros2 param get /zed_zupt_filter_node enable_zupt\n'
            '========================================\n'
        ))]
    ))

    return actions


def generate_launch_description():
    """Generate launch description"""

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed',
            description='Name of the camera namespace'
        ),

        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2i',
            description='ZED camera model'
        ),

        DeclareLaunchArgument(
            'enable_visualization',
            default_value='false',
            description='Enable RViz2 visualization',
            choices=['true', 'false']
        ),

        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level',
            choices=['debug', 'info', 'warn', 'error']
        ),

        # Opaque function for sequenced launch
        OpaqueFunction(function=launch_setup)
    ])
