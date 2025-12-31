# Eclevive Robot - ZED2i + nvblox + Nav2 Autonomous Navigation System

A robot equipped with mecanum wheels and a robot arm. A complete autonomous navigation system based on ROS2 Humble. Integrates 3D mapping with NVIDIA Isaac ROS nvblox, ZED2i Visual SLAM, and Nav2 path planning.

## System Overview

- **Platform**: Jetson Orin Nano / AGX Orin
- **ROS2 Version**: Humble
- **Robot Type**: Mecanum wheel mobile robot
- **Primary Sensors**: ZED2i, LiDAR x2, IMU
- **Control Method**: PS5 controller + autonomous navigation

## Key Features

1. **3D Mapping**: NVIDIA Isaac ROS nvblox (ESDF, extended range 14m)
2. **Localization**: ZED2i Visual SLAM (without AMCL)
3. **Path Planning**: Navigation2 (Theta* Planner + MPPI Controller)
4. **Obstacle Avoidance**: LiDAR + ZED Depth fusion costmap
5. **Human Recognition**: ZED Body Tracking + gesture control

## Quick Start

### 1. Install Dependencies

```bash
cd ~/ros2_ws
./setup_dependencies.sh
```

### 2. Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-skip-build-finished --symlink-install \
  --packages-skip isaac_ros_ess_models_install \
                  isaac_ros_peoplenet_models_install \
                  isaac_ros_peoplesemseg_models_install \
                  isaac_ros_nova_recorder
```

### 3. Launch

```bash
source install/setup.bash
ros2 launch bringup zed2i_nvblox_nav2_launch.py
```

## Package Structure

### Custom Packages (included in this repository)

| Package | Description |
|---------|-------------|
| `bringup` | Integrated launch files, configuration files |
| `mark3_urdf` | Robot URDF definition |
| `joy_mecanum_controller` | PS5 controller operation |
| `zed_goal_publisher` | Goal setting via ZED gestures |
| `zed_human_tracker` | Human following function |
| `zed_zupt_wrapper` | ZED ZUPT (Zero Velocity Update) filter |
| `safety_sensor` | Safety sensor integration |
| `cmd_vel_float_changer` | Velocity command conversion |
| `odom_publisher` | Odometry publisher |
| `zed_gesture_controller` | Gesture control |
| `zed_imu_republisher` | IMU data republisher |
| `zed_odom_watcher` | Odometry monitoring |

### External Dependencies (install separately)

- **NVIDIA Isaac ROS**: nvblox, Visual SLAM, Image Pipeline
- **Navigation2**: Path planning, autonomous navigation
- **ZED SDK**: ZED2i camera driver
- **LiDAR**: ldlidar_stl_ros2, sllidar_ros2
- **Others**: slam_toolbox, laser_filters, pointcloud_to_laserscan

See [setup_dependencies.sh](setup_dependencies.sh) for details.

## System Architecture

### TF Tree Structure

```
map (nvblox global_frame)
 └─ odom (ZED2i map_frame - Visual SLAM parent frame)
     └─ zed_camera_origin (ZED2i odometry_frame)
         └─ zed_camera_link (robot reference point)
             ├─ base_link
             ├─ front_lidar
             └─ back_lidar
```

**Important**: The ZED2i `map_frame='odom'` setting is **absolutely prohibited from being changed**. This setting is essential for both Visual Odometry and nvblox Occupancy Grid publishing to work together.

### Key Topics

- LiDAR: `/scan_filtered` (merged)
- Costmap: `/local_costmap/costmap_raw`, `/global_costmap/costmap_raw`
- Odometry: `/zed/zed_node/odom`
- Navigation: `/goal_pose`, `/cmd_vel`

See [CLAUDE.md](CLAUDE.md) for details.

## Performance Optimization

### Local Costmap Optimization (2025-11-03)

- **Update Frequency**: 25Hz → 30Hz (measured 11.8Hz)
- **Resolution**: 0.04m → 0.03m (higher resolution)
- **Obstacle Detection Range**: 1.8m → 2.5m (+39%)
- **LiDAR Layer**: VoxelLayer → ObstacleLayer (lighter weight)

Result: Obstacle detection accuracy improved by 77%, CPU load within acceptable range (52% idle)

## Troubleshooting

### Check LiDAR Physical Mounting

**Symptom**: Unable to generate map properly with SLAM
**Cause**: LiDAR is tilted downward
**Solution**: Physically adjust LiDAR mounting angle to be horizontal

### Check TF Configuration

```bash
ros2 param get /zed/zed_node pos_tracking.publish_map_tf  # Expected: true
ros2 param get /zed/zed_node pos_tracking.map_frame       # Expected: odom
```

See [docs/troubleshooting.md](.claude/docs/troubleshooting.md) for details.

## Documentation

- **[CLAUDE.md](CLAUDE.md)**: Basic configuration and overview
- **[system-architecture.md](.claude/docs/system-architecture.md)**: System architecture details
- **[tf-tree.md](.claude/docs/tf-tree.md)**: TF tree specification
- **[topics.md](.claude/docs/topics.md)**: Topic specification
- **[build-system.md](.claude/docs/build-system.md)**: Build system
- **[troubleshooting.md](.claude/docs/troubleshooting.md)**: Troubleshooting

## Slash Commands

```bash
/ros2-diagnose     # Run system diagnostics
/ros2-build        # Build packages
/ros2-test-nav     # Run navigation tests
```

## License

Custom packages: MIT License

External packages follow their respective licenses:
- NVIDIA Isaac ROS: Apache-2.0
- Navigation2: Apache-2.0
- ZED SDK: Stereolabs License

## Author

- GitHub: [Your GitHub Account]
- Development Environment: Jetson Orin Nano/AGX Orin + ROS2 Humble

## Acknowledgments

This project uses the following open source projects:
- [NVIDIA Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS)
- [Navigation2](https://github.com/ros-planning/navigation2)
- [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
