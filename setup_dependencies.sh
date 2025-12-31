#!/bin/bash
# ==============================================================================
# ROS2 Mecanum Robot - Dependency Package Setup Script
# ==============================================================================

set -e

echo "=========================================="
echo "ROS2 Mecanum Robot Dependency Package Setup"
echo "=========================================="

# Workspace directory
WS_DIR="$HOME/ros2_ws"
SRC_DIR="$WS_DIR/src"

cd "$SRC_DIR"

# ==============================================================================
# 1. NVIDIA Isaac ROS
# ==============================================================================
echo ""
echo "=== NVIDIA Isaac ROS ==="
echo "Manual installation required:"
echo "  https://nvidia-isaac-ros.github.io/getting_started/index.html"
echo ""
echo "Required packages:"
echo "  - isaac_ros_nvblox"
echo "  - isaac_ros_visual_slam"
echo "  - isaac_ros_common"
echo "  - isaac_ros_image_pipeline"
echo ""

# ==============================================================================
# 2. Navigation2
# ==============================================================================
echo ""
echo "=== Navigation2 ==="
if [ ! -d "navigation2" ]; then
  echo "Cloning navigation2..."
  git clone -b humble https://github.com/ros-planning/navigation2.git
else
  echo "✓ navigation2 already exists"
fi

# ==============================================================================
# 3. ZED ROS2 Wrapper
# ==============================================================================
echo ""
echo "=== ZED ROS2 Wrapper ==="
if [ ! -d "zed-ros2-wrapper" ]; then
  echo "Cloning zed-ros2-wrapper..."
  git clone -b humble https://github.com/stereolabs/zed-ros2-wrapper.git
else
  echo "✓ zed-ros2-wrapper already exists"
fi

if [ ! -d "zed-ros2-interfaces" ]; then
  echo "Cloning zed-ros2-interfaces..."
  git clone -b humble https://github.com/stereolabs/zed-ros2-interfaces.git
else
  echo "✓ zed-ros2-interfaces already exists"
fi

if [ ! -d "zed-ros2-examples" ]; then
  echo "Cloning zed-ros2-examples..."
  git clone -b humble https://github.com/stereolabs/zed-ros2-examples.git
else
  echo "✓ zed-ros2-examples already exists"
fi

# ==============================================================================
# 4. SLAM Toolbox
# ==============================================================================
echo ""
echo "=== SLAM Toolbox ==="
if [ ! -d "slam_toolbox" ]; then
  echo "Cloning slam_toolbox..."
  git clone -b humble https://github.com/SteveMacenski/slam_toolbox.git
else
  echo "✓ slam_toolbox already exists"
fi

# ==============================================================================
# 5. LiDAR Drivers
# ==============================================================================
echo ""
echo "=== LiDAR Drivers ==="
if [ ! -d "ldlidar_stl_ros2" ]; then
  echo "Cloning ldlidar_stl_ros2..."
  git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
else
  echo "✓ ldlidar_stl_ros2 already exists"
fi

if [ ! -d "sllidar_ros2" ]; then
  echo "Cloning sllidar_ros2..."
  git clone -b humble https://github.com/Slamtec/sllidar_ros2.git
else
  echo "✓ sllidar_ros2 already exists"
fi

# ==============================================================================
# 6. Laser Filters & Tools
# ==============================================================================
echo ""
echo "=== Laser Filters & Tools ==="
if [ ! -d "laser_filters" ]; then
  echo "Cloning laser_filters..."
  git clone -b humble https://github.com/ros-perception/laser_filters.git
else
  echo "✓ laser_filters already exists"
fi

if [ ! -d "pointcloud_to_laserscan" ]; then
  echo "Cloning pointcloud_to_laserscan..."
  git clone -b humble https://github.com/ros-perception/pointcloud_to_laserscan.git
else
  echo "✓ pointcloud_to_laserscan already exists"
fi

# ==============================================================================
# 7. Odometry
# ==============================================================================
echo ""
echo "=== Odometry ==="
if [ ! -d "rf2o_laser_odometry" ]; then
  echo "Cloning rf2o_laser_odometry..."
  git clone -b humble https://github.com/MAPIRlab/rf2o_laser_odometry.git
else
  echo "✓ rf2o_laser_odometry already exists"
fi

# ==============================================================================
# 8. micro-ROS
# ==============================================================================
echo ""
echo "=== micro-ROS ==="
if [ ! -d "micro_ros_setup" ]; then
  echo "Cloning micro_ros_setup..."
  git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git
else
  echo "✓ micro_ros_setup already exists"
fi

# ==============================================================================
# 9. Additional Tools
# ==============================================================================
echo ""
echo "=== Additional Tools ==="
if [ ! -d "ros2_rs_pcl" ]; then
  echo "Cloning ros2_rs_pcl..."
  git clone https://github.com/ROBOTIS-GIT/ros2_rs_pcl.git
else
  echo "✓ ros2_rs_pcl already exists"
fi

# ==============================================================================
# Install dependency packages
# ==============================================================================
echo ""
echo "=========================================="
echo "rosdep Dependency Package Installation"
echo "=========================================="

cd "$WS_DIR"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Install ZED SDK: https://www.stereolabs.com/developers/release"
echo "  2. Install NVIDIA Isaac ROS (see above)"
echo "  3. Build: colcon build --symlink-install"
echo ""
