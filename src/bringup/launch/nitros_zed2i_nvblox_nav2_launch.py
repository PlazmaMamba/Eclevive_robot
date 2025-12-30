#!/usr/bin/env python3
"""
ZED2i + nvblox + Nav2 Integrated Launch File (NITROS Compatible)

This launch file starts a complete autonomous navigation system utilizing NITROS (Isaac ROS high-speed communication):
- ZED2i camera (Visual SLAM, human skeleton detection) with NITROS
- NVIDIA Isaac ROS nvblox (3D mapping, extended range 14m) with NITROS
- Navigation2 stack (autonomous navigation)
- Front/Back LiDAR (obstacle detection)
- PS5 controller (teleoperation)
- All devices and sensors

Usage:
  # NITROS enabled (default, recommended)
  ros2 launch bringup nitros_zed2i_nvblox_nav2_launch.py

  # IPC enabled (NITROS disabled, legacy mode)
  ros2 launch bringup nitros_zed2i_nvblox_nav2_launch.py enable_ipc:=true

  # With visualization
  ros2 launch bringup nitros_zed2i_nvblox_nav2_launch.py enable_visualization:=true

Notes:
  - NITROS enabled by default (enable_ipc:=false)
  - enable_ipc:=false enables NITROS (Isaac ROS high-speed communication, can be omitted)
  - enable_ipc:=true enables legacy IPC communication (compatibility priority)
  - NITROS accelerates image transfer between ZED→nvblox (GPU Direct)
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
    enable_ipc = LaunchConfiguration('enable_ipc', default='false')

    # ========================================
    # 1. ZED2i + nvblox + All Devices Launch (with enable_ipc parameter)
    # ========================================
    # Launch all sensors, nvblox, and devices (NITROS compatible)
    zed_nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'nitros_zed2i_nvblox_fixed.launch.py')
        ),
        launch_arguments={
            'enable_visualization': enable_visualization,
            'enable_ipc': enable_ipc,
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
        '(NITROS Compatible)\n'
        '========================================\n'
        '\nLaunching Components:\n'
        '  ✓ ZED2i Camera (Visual Odometry + Body Tracking) [NITROS]\n'
        '  ✓ ZED ZUPT Filter (Odometry Stabilization)\n'
        '  ✓ nvblox (3D Mapping, Extended Range: 14m) [NITROS]\n'
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
        '\nNITROS Configuration:\n'
        '  - enable_ipc=false: NITROS enabled (Isaac ROS high-speed communication)\n'
        '  - enable_ipc=true: IPC enabled (legacy mode)\n'
        '  - Accelerates image transfer between ZED→nvblox (GPU Direct)\n'
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
            'System Launch Complete! (NITROS Version)\n'
            '========================================\n'
            '\nVerification Commands:\n'
            '  # Check TF\n'
            '  ros2 run tf2_tools view_frames\n'
            '  ros2 run tf2_ros tf2_echo map zed_camera_link\n'
            '\n  # Check topics\n'
            '  ros2 topic hz /nvblox_node/esdf_pointcloud\n'
            '  ros2 topic hz /scan_filtered\n'
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
            '\n  # Check NITROS operation\n'
            '  ros2 topic info /zed/zed_node/rgb/image_rect_color/nitros\n'
            '  ros2 topic info /zed/zed_node/depth/depth_registered/nitros\n'
            '\nHow to Start Navigation:\n'
            '  1. Set goal in RViz2\n'
            '  2. Or use ZED Goal Publisher with gesture commands\n'
            '  3. Or publish directly to /goal_pose topic\n'
            '\nObstacle Avoidance:\n'
            '  - Collision Monitor automatically detects obstacles\n'
            '  - Within 0.25m: Emergency stop\n'
            '  - Within 0.45m: 40% deceleration\n'
            '  - 1.5s ahead: Dynamic predictive avoidance\n'
            '\nNITROS Performance:\n'
            '  - Accelerated image transfer via GPU Direct communication\n'
            '  - Reduced CPU load\n'
            '  - Improved latency\n'
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
        DeclareLaunchArgument(
            'enable_ipc',
            default_value='false',
            description='Enable IPC (Intra-Process Communication). Set to false for NITROS (recommended)',
            choices=['true', 'false']
        ),

        # Launch components
        startup_log,
        zed_nvblox_launch,
        nav2_launch,
        status_log,
    ])
