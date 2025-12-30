#!/usr/bin/env python3
"""
ZED2i + nvblox + Nav2 Integrated Launch File

This launch file starts a complete autonomous navigation system:
- ZED2i camera (Visual SLAM, human skeleton detection)
- NVIDIA Isaac ROS nvblox (3D mapping, extended range 14m)
- Navigation2 stack (autonomous navigation)
- Front/Back LiDAR (obstacle detection)
- PS5 controller (teleoperation)
- All devices and sensors

Usage:
  ros2 launch bringup zed2i_nvblox_nav2_launch.py

Notes:
  - AMCL is not used (nvblox provides map -> zed_odom TF)
  - Localization realized by ZED Visual Odometry + nvblox
  - Costmap generated from nvblox ESDF pointcloud and LiDAR scans
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description"""

    # Package directories
    bringup_dir = get_package_share_directory('bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(nav2_bringup_dir, 'params', 'my_nav2_params_mppi1.yaml')
    )
    autostart = LaunchConfiguration('autostart', default='true')
    enable_visualization = LaunchConfiguration('enable_visualization', default='false')

    # ========================================
    # 1. ZED2i + nvblox + All Devices Launch
    # ========================================
    # Launch all sensors, nvblox, and devices
    zed_nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'zed2i_nvblox_fixed.launch.py')
        ),
        launch_arguments={
            'enable_visualization': enable_visualization,
        }.items()
    )

    # ========================================
    # 2. Navigation2 Launch (without AMCL and Map Server)
    # ========================================
    # Launch Nav2 stack (excluding AMCL and Map Server)
    # AMCL is not needed because nvblox provides map -> zed_odom TF
    nav2_launch = TimerAction(
        period=8.0,  # Wait until ZED and nvblox are fully started
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'my_navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                }.items()
            )
        ]
    )

    # ========================================
    # Log Messages
    # ========================================
    startup_log = LogInfo(msg=TextSubstitution(text=
        '\n========================================\n'
        'ZED2i + nvblox + Nav2 Integrated System Launch\n'
        '========================================\n'
        '\nLaunching Components:\n'
        '  ✓ ZED2i Camera (Visual Odometry + Body Tracking)\n'
        '  ✓ ZED ZUPT Filter (Odometry Stabilization)\n'
        '  ✓ nvblox (3D Mapping, Extended Range: 14m)\n'
        '  ✓ LiDAR Front/Back + Merger + Filter\n'
        '  ✓ PS5 Controller + Mecanum Control\n'
        '  ✓ micro-ROS Agent\n'
        '  ✓ ZED Goal Publisher\n'
        '  ✓ Safety Sensor\n'
        '  ✓ Navigation2 Stack (Autonomous Navigation)\n'
        '    - Collision Monitor (Enhanced Obstacle Avoidance)\n'
        '      - Stop Zone: 0.25m (Emergency Stop)\n'
        '      - Slowdown Zone: 0.45m (40% Deceleration)\n'
        '      - Approach Zone: 1.5s Forward Prediction\n'
        '\nCoordinate Frame Structure:\n'
        '  map (nvblox global_frame)\n'
        '    └─ odom (ZED2i map_frame - Visual SLAM parent frame)\n'
        '        └─ zed_camera_origin (ZED2i odometry_frame)\n'
        '            └─ zed_camera_link (Robot reference)\n'
        '                ├─ base_link\n'
        '                ├─ front_lidar\n'
        '                └─ back_lidar\n'
        '\nLocalization:\n'
        '  - AMCL is not used\n'
        '  - High-precision pose estimation with ZED Visual Odometry\n'
        '  - ZED2i settings: map_frame: odom, odometry_frame: zed_camera_origin\n'
        '  - map → odom: static_transform_publisher (fixed, 0 0 0)\n'
        '\nCostmap Sources:\n'
        '  - Global: nvblox ESDF pointcloud + LiDAR scan\n'
        '  - Local: ZED pointcloud + LiDAR scan\n'
        '========================================\n'
    ))

    status_log = TimerAction(
        period=10.0,
        actions=[LogInfo(msg=TextSubstitution(text=
            '\n========================================\n'
            'System Launch Complete!\n'
            '========================================\n'
            '\nVerification Commands:\n'
            '  # Check TF\n'
            '  ros2 run tf2_tools view_frames\n'
            '  ros2 run tf2_ros tf2_echo map zed_camera_link\n'
            '\n  # Check topics\n'
            '  ros2 topic hz /nvblox_node/esdf_pointcloud\n'
            '  ros2 topic hz /merged_scan_filtered\n'
            '  ros2 topic hz /cmd_vel\n'
            '\n  # Check nodes\n'
            '  ros2 node list | grep -E "nvblox|controller|planner|collision"\n'
            '\n  # Check costmaps\n'
            '  ros2 topic echo /global_costmap/costmap --once\n'
            '  ros2 topic echo /local_costmap/costmap --once\n'
            '\n  # Check Collision Monitor\n'
            '  ros2 topic hz /cmd_vel_monitored\n'
            '  ros2 topic echo /polygon_stop\n'
            '  ros2 topic echo /polygon_slowdown\n'
            '\nHow to Start Navigation:\n'
            '  1. Set goal in RViz2\n'
            '  2. Or use ZED Goal Publisher with gesture commands\n'
            '  3. Or publish directly to /goal_pose topic\n'
            '\nObstacle Avoidance:\n'
            '  - Collision Monitor automatically detects obstacles\n'
            '  - Within 0.25m: Emergency stop\n'
            '  - Within 0.45m: 40% deceleration\n'
            '  - 1.5s ahead: Dynamic predictive avoidance\n'
            '========================================\n'
        ))]
    )

    # Create launch description
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav2_bringup_dir, 'params', 'my_nav2_params_mppi1.yaml'),
            description='Full path to the ROS2 parameters file to use'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='false',
            description='Enable RViz2 visualization',
            choices=['true', 'false']
        ),

        # Launch components
        startup_log,
        zed_nvblox_launch,
        nav2_launch,
        status_log,
    ])
