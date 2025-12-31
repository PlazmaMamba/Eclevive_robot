# ZED ZUPT Wrapper

A package that applies a ZUPT (Zero-Velocity Update) filter to ZED2i Visual Odometry to prevent odometry drift and instability.

## Problem

ZED2i Visual Odometry becomes unstable and drifts uncontrollably in the following situations:

1. **Dark environments**: Camera feed is dark, insufficient feature points
2. **Close to walls**: Narrow field of view, difficult to track feature points
3. **Lack of texture**: Monotonous walls where feature points cannot be found

→ **Even when the robot is stationary, odometry drifts rapidly as if moving across a plane**

## Solution: ZUPT Theory

ZUPT (Zero-Velocity Update) principle:

```
IF (IMU acceleration ≈ 0) AND (IMU angular velocity ≈ 0):
    → Robot is stationary

    IF Visual Odometry reports movement:
        → Correct as anomaly (set velocity to zero)
```

### Algorithm Details

1. **Stationary Detection**:
   - Store IMU data in a window buffer (e.g., 15 samples)
   - Acceleration norm < 0.5 m/s² AND angular velocity norm < 0.1 rad/s
   - If 70% or more of the window meets conditions → Stationary

2. **Odometry Correction**:
   - **When stationary**: Force velocity to zero, fix position to previous value
   - **When moving**: Limit sudden velocity changes (jump suppression)

3. **Smoothing**:
   - Smooth velocity changes
   - Prevent abnormal acceleration due to drift

## Usage

### Basic Launch

```bash
# Launch ZED camera (separate terminal)
ros2 launch zed_wrapper zed_camera.launch.py

# Launch ZUPT filter
ros2 launch zed_zupt_wrapper zed_zupt_filter.launch.py
```

### Topics

#### Input

- `/zed/zed_node/odom` - ZED Visual Odometry (raw data)
- `/zed/zed_node/imu/data` - ZED IMU data (400Hz)

#### Output

- `/zed/zed_node/odom_zupt` - ZUPT-filtered odometry (recommended)
- `/zupt/status` - ZUPT status (for debugging)

### Parameter Tuning

[config/zed_zupt_params.yaml](config/zed_zupt_params.yaml)

#### Stationary Detection Parameters

```yaml
# Acceleration threshold (m/s²)
# Low → Strict stationary detection, High → Loose stationary detection
zupt_accel_threshold: 0.5

# Angular velocity threshold (rad/s)
zupt_gyro_threshold: 0.1

# Window size (number of samples)
# Small → Faster response, Large → More stable
zupt_window_size: 15

# Confidence threshold (0.0-1.0)
# High → Correct only when stationary is certain
zupt_confidence_threshold: 0.7
```

#### Correction Parameters

```yaml
# Maximum velocity jump (m/s)
# Limit sudden velocity changes due to drift
max_velocity_jump: 0.5

# Maximum angular velocity jump (rad/s)
max_angular_jump: 1.0
```

## Integration into zed2i_nvblox_fixed.launch.py

To integrate into the main launch file:

```python
# Add ZUPT filter node
zed_zupt_dir = get_package_share_directory('zed_zupt_wrapper')
zed_zupt_config = os.path.join(zed_zupt_dir, 'config', 'zed_zupt_params.yaml')

zed_zupt_node = Node(
    package='zed_zupt_wrapper',
    executable='zed_zupt_filter_node',
    name='zed_zupt_filter_node',
    output='screen',
    parameters=[zed_zupt_config],
    emulate_tty=True,
)

# Execute after ZED camera launch (e.g., after 3 seconds)
load_zed_zupt = TimerAction(
    period=3.0,
    actions=[zed_zupt_node]
)
actions.append(load_zed_zupt)
```

Then, change the odometry used by nvblox and navigation:

```python
# nvblox remapping
nvblox_remappings = [
    # ...
    ('pose', '/zed/zed_node/odom_zupt'),  # Use ZUPT-filtered odometry
]
```

## Debugging

### Check ZUPT Status

```bash
ros2 topic echo /zupt/status
```

Output:
```yaml
linear:
  x: 1.0  # 1.0 = Stationary, 0.0 = Moving
  y: 0.23 # Average acceleration (m/s²)
  z: 0.05 # Average angular velocity (rad/s)
angular:
  x: 142.0  # ZUPT correction count
```

### Compare Odometry

```bash
# Original odometry
ros2 topic echo /zed/zed_node/odom

# ZUPT-filtered
ros2 topic echo /zed/zed_node/odom_zupt

# Check difference (separate terminal)
ros2 run plotjuggler plotjuggler
```

### Dynamic Parameter Changes

```bash
# Disable ZUPT (for testing)
ros2 param set /zed_zupt_filter_node enable_zupt false

# Change threshold
ros2 param set /zed_zupt_filter_node zupt_accel_threshold 0.3
```

## Troubleshooting

### Issue 1: ZUPT is too aggressive, actual movement is also stopped

**Cause**: Thresholds are too high

**Solution**:
```yaml
zupt_accel_threshold: 0.3  # Decrease from 0.5 → 0.3
zupt_gyro_threshold: 0.05  # Decrease from 0.1 → 0.05
```

### Issue 2: Odometry still drifts

**Cause**: Thresholds are too low, or confidence is too low

**Solution**:
```yaml
zupt_accel_threshold: 0.7  # Increase from 0.5 → 0.7
zupt_confidence_threshold: 0.8  # Increase from 0.7 → 0.8
zupt_window_size: 20  # Increase from 15 → 20
```

### Issue 3: Response is slow

**Cause**: Window size is too large

**Solution**:
```yaml
zupt_window_size: 10  # Decrease from 15 → 10
```

## Technical Details

### ZUPT Detection Algorithm

```cpp
// 1. Extract acceleration and angular velocity from IMU buffer
for (const auto& imu : imu_buffer_) {
    // Gravity compensation
    accel_norm = sqrt(ax² + ay² + (az - 9.81)²);
    gyro_norm = sqrt(gx² + gy² + gz²);

    // Threshold check
    if (accel_norm < threshold_accel && gyro_norm < threshold_gyro) {
        stationary_count++;
    }
}

// 2. Calculate confidence
confidence = stationary_count / buffer_size;

// 3. Determine if stationary
is_stationary = (confidence >= confidence_threshold);
```

### Odometry Correction

```cpp
if (is_stationary) {
    // When stationary: Zero velocity, fix position
    odom.twist.twist.linear = {0, 0, 0};
    odom.twist.twist.angular = {0, 0, 0};
    odom.pose.pose = last_pose;  // Fix position to previous value
} else {
    // When moving: Limit jumps
    odom.twist.twist.linear.x = limit_jump(
        new_velocity, old_velocity, max_jump
    );
}
```

## Performance

- **CPU Usage**: Approximately 1-2% (running with ZED IMU at 400Hz)
- **Latency**: Approximately 25-50ms (depends on window size)
- **Memory**: Approximately 10MB

## References

- Skog, I., et al. "Zero-velocity detection—An algorithm evaluation." IEEE Transactions on Biomedical Engineering (2010)
- Jimenez, A. R., et al. "A comparison of Pedestrian Dead-Reckoning algorithms using a low-cost MEMS IMU." (2009)

---

**Created**: 2025-10-19
**Version**: 1.0.0
**Supported ZED Model**: ZED2i
**ROS2 Version**: Humble
