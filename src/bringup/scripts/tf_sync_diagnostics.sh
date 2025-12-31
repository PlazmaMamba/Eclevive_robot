#!/bin/bash

# TF-Topic Synchronization Diagnostic Script
# Usage: ./tf_sync_diagnostics.sh [duration_seconds]

DURATION=${1:-30}  # Default 30 seconds

echo "=== TF-Topic Synchronization Diagnostics Started ==="
echo "Diagnostic Duration: ${DURATION} seconds"
echo "Start Time: $(date)"
echo

# 1. Check current TF tree structure
echo "1. TF Tree Structure:"
echo "======================================"
timeout 5s ros2 run tf2_tools view_frames || echo "Warning: TF frames could not be retrieved"
echo

# 2. Active topic list
echo "2. Related Topic List:"
echo "======================================"
echo "Odometry topics:"
ros2 topic list | grep -E "(odom|odometry)" || echo "No odometry topics found"
echo "Scan topics:"
ros2 topic list | grep -E "(scan|laser)" || echo "No scan topics found"
echo "TF topics:"
ros2 topic list | grep -E "(tf|transform)" || echo "No tf topics found"
echo

# 3. TF transform delay check
echo "3. TF Transform Delay Test:"
echo "======================================"
echo "Testing map -> zed_camera_link transform..."
ros2 run tf2_ros tf2_echo map zed_camera_link &
TF_ECHO_PID=$!
sleep 5
kill $TF_ECHO_PID 2>/dev/null || true
echo

echo "Testing zed_odom -> zed_camera_link transform..."
ros2 run tf2_ros tf2_echo zed_odom zed_camera_link &
TF_ECHO_PID=$!
sleep 5
kill $TF_ECHO_PID 2>/dev/null || true
echo

# 4. Topic timestamp synchronization check
echo "4. Topic Timestamp Synchronization Analysis:"
echo "======================================"
echo "Odometry timestamp check:"
timeout 10s ros2 topic echo --once /zed/zed_node/odom --field header.stamp 2>/dev/null || echo "No odometry data received"

echo "Scan timestamp check:"
timeout 10s ros2 topic echo --once /scan_filtered --field header.stamp 2>/dev/null || echo "No scan data received"

echo "Current ROS time:"
timeout 5s ros2 param get /use_sim_time use_sim_time 2>/dev/null || echo "Could not retrieve sim_time setting"
echo

# 5. Sync coordinator status check
echo "5. TF Sync Coordinator Status:"
echo "======================================"
if ros2 node list | grep -q "tf_sync_coordinator"; then
    echo "✓ TF sync coordinator is running"
    echo "Retrieving statistics..."
    timeout 10s ros2 topic echo --once /rosout --field msg | grep -i "sync" | tail -5 || echo "Could not retrieve sync statistics"
else
    echo "✗ TF sync coordinator not found"
fi
echo

# 6. Navigation2 node status check
echo "6. Navigation2 Node Status:"
echo "======================================"
NAV_NODES=("planner_server" "controller_server" "bt_navigator" "behavior_server")
for node in "${NAV_NODES[@]}"; do
    if ros2 node list | grep -q $node; then
        echo "✓ $node: Running"
    else
        echo "✗ $node: Stopped"
    fi
done
echo

# 7. Real-time diagnostics (specified duration)
echo "7. Real-time Timestamp Sync Monitoring (${DURATION} seconds):"
echo "======================================"
echo "Press Ctrl+C to interrupt..."

# Monitor timestamp of each topic in background
{
    while true; do
        ODOM_TIME=$(timeout 2s ros2 topic echo --once /zed/zed_node/odom --field header.stamp.sec 2>/dev/null || echo "N/A")
        SCAN_TIME=$(timeout 2s ros2 topic echo --once /scan_filtered --field header.stamp.sec 2>/dev/null || echo "N/A")
        CURRENT_TIME=$(date +%s)

        if [[ "$ODOM_TIME" != "N/A" && "$SCAN_TIME" != "N/A" ]]; then
            TIME_DIFF=$((ODOM_TIME - SCAN_TIME))
            ODOM_DELAY=$((CURRENT_TIME - ODOM_TIME))
            SCAN_DELAY=$((CURRENT_TIME - SCAN_TIME))

            echo "$(date '+%H:%M:%S') - Odom: ${ODOM_TIME}s (delay: ${ODOM_DELAY}s), Scan: ${SCAN_TIME}s (delay: ${SCAN_DELAY}s), Diff: ${TIME_DIFF}s"
        else
            echo "$(date '+%H:%M:%S') - Data unavailable (Odom: $ODOM_TIME, Scan: $SCAN_TIME)"
        fi

        sleep 3
    done
} &
MONITOR_PID=$!

# Terminate monitoring after specified time
sleep $DURATION
kill $MONITOR_PID 2>/dev/null || true
wait $MONITOR_PID 2>/dev/null || true

echo
echo "=== Diagnostics Complete ==="
echo "End Time: $(date)"

# Display recommended countermeasures
echo
echo "=== Recommended Countermeasures ==="
echo "1. If timestamp deviation is large:"
echo "   - Check system time synchronization"
echo "   - Adjust transform_tolerance value"
echo "   - Increase TF sync coordinator tolerance"
echo
echo "2. If data cannot be retrieved:"
echo "   - Check sensor node startup status"
echo "   - Check topic name inconsistencies"
echo "   - Verify network connection"
echo
echo "3. If Navigation2 errors persist:"
echo "   - Check QoS settings compatibility"
echo "   - Try restarting nodes"
echo "   - Check detailed errors in log files"