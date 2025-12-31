# zed_goal_publisher

Robot control package using human recognition and gesture control with ZED2i camera

## Overview

This package recognizes human skeleton using the ZED2i stereo camera and controls a mecanum wheel robot based on gestures.

## Features

### Gesture Recognition
- **Raise both hands above shoulders** → Move forward
- **Bend and raise right hand** → Turn right
- **Bend and raise left hand** → Turn left
- **Extend right hand horizontally** → Strafe right
- **Extend left hand horizontally** → Strafe left

## Launch Instructions

### Using Launch File (Recommended)

```bash
ros2 launch zed_goal_publisher zed_goal_publisher.launch.py
```

### Direct Execution

```bash
ros2 run zed_goal_publisher zed_goal_publisher --ros-args --params-file $(ros2 pkg prefix zed_goal_publisher)/share/zed_goal_publisher/config/zed_goal_publisher.yaml
```

## Parameter Configuration

Parameters can be configured in `config/zed_goal_publisher.yaml`.

### Topic Name Configuration

```yaml
skeleton_topic: "zed/zed_node/body_trk/skeletons"  # ZED skeleton tracking topic
target_arm_pose_topic: "target_arm_pose"            # Arm target pose
current_arm_pose_topic: "current_arm_pose"          # Arm current pose
cmd_vel_topic: "cmd_vel"                            # Velocity command output
goal_pose_topic: "goal_pose"                        # Goal position
zed_goal_status_topic: "zed_goal_status"            # Goal status
```

### Velocity Parameters

#### Forward Movement (Both Hands Raised)
```yaml
forward_vel_fast: 0.5    # Forward high speed (m/s)
forward_vel_slow: 0.1    # Forward low speed (m/s)
```

#### Turn Right (Right Hand Bent)
```yaml
turn_right_vel_fast: 1.6   # Turn right high speed (rad/s)
turn_right_vel_slow: 0.05  # Turn right low speed (rad/s)
```

#### Turn Left (Left Hand Bent)
```yaml
turn_left_vel_fast: -1.6   # Turn left high speed (rad/s)
turn_left_vel_slow: -0.05  # Turn left low speed (rad/s)
```

#### Strafe Right (Right Hand Horizontal)
```yaml
strafe_right_vel_fast: 0.70   # Strafe right high speed (m/s)
strafe_right_vel_slow: 0.10   # Strafe right low speed (m/s)
```

#### Strafe Left (Left Hand Horizontal)
```yaml
strafe_left_vel_fast: -0.70   # Strafe left high speed (m/s)
strafe_left_vel_slow: -0.10   # Strafe left low speed (m/s)
```

## Parameter Adjustment Instructions

1. Edit `config/zed_goal_publisher.yaml`
2. Modify velocity values (example: reduce values if you want slower movement)
3. Build the package (not required if using symlink-install)

```bash
colcon build --packages-select zed_goal_publisher --symlink-install
```

4. Restart

```bash
ros2 launch zed_goal_publisher zed_goal_publisher.launch.py
```

## Topics

### Subscribe
- `/zed/zed_node/body_trk/skeletons` (zed_msgs/msg/ObjectsStamped) - Skeleton tracking data
- `/target_arm_pose` (std_msgs/msg/Int32MultiArray) - Arm target pose

### Publish
- `/cmd_vel` (geometry_msgs/msg/Twist) - Velocity command
- `/goal_pose` (geometry_msgs/msg/PoseStamped) - Navigation goal
- `/target_arm_pose` (std_msgs/msg/Int32MultiArray) - Arm target pose
- `/current_arm_pose` (std_msgs/msg/Int32MultiArray) - Arm current pose
- `/zed_goal_status` (std_msgs/msg/Int32) - Goal status

## File Structure

```
zed_goal_publisher/
├── src/
│   └── zed_goal_publisher.cpp      # Main program
├── launch/
│   └── zed_goal_publisher.launch.py # Launch file
├── config/
│   └── zed_goal_publisher.yaml      # Parameter configuration
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Dependencies

- rclcpp
- rclcpp_action
- std_msgs
- zed_msgs
- geometry_msgs
- nav2_msgs
- tf2
- tf2_ros
- tf2_geometry_msgs
- sensor_msgs

## Revision History

### 2025-10-12
- Changed to allow topic names to be configured from YAML parameters
- Parameterized velocity values (linear.x, linear.y, angular.z) in YAML
- Added launch file
- Added parameter file
