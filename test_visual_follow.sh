#!/bin/bash
# Visual Following Pipeline Test
# Tests: target_tracker + following_controller + return_home + waypoint_patrol
set -e

source /opt/ros/humble/setup.bash
source ~/Documents/ros2-robt/ros2_ws/install/setup.bash

echo "=== Launching Full System ==="
ros2 launch mapless_nav mapless_nav.launch.py \
    use_nav2:=true \
    enable_patrol:=true \
    enable_visualization:=true \
    odom_mode:=fake &

LAUNCH_PID=$!
sleep 15

echo ""
echo "=== Node Status ==="
ros2 node list 2>/dev/null | grep -E "tracker|following|return_home|patrol|controller|fusion"

echo ""
echo "=== Target Subscription Check ==="
ros2 topic info /target_tracker/primary_target 2>/dev/null | grep -E "Publisher|Subscription"

echo ""
echo "=== Simulating Target at 1.5m ==="
ros2 topic pub /target_tracker/primary_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}}}" \
  -r 10 &
TARGET_PID=$!

sleep 3

echo ""
echo "=== Following cmd_vel ==="
ros2 topic echo /cmd_vel --timeout 3 2>/dev/null | head -10 || echo "(no cmd_vel output)"

echo ""
echo "=== Return Home Test ==="
ros2 service call /return_home std_srvs/srv/Trigger 2>/dev/null

echo ""
echo "=== Nav2 Topics ==="
ros2 topic list 2>/dev/null | grep -E "plan|costmap|navigate" | head -10

echo ""
echo "=== Test Complete ==="
echo "PID: $LAUNCH_PID (kill to stop)"
echo "Target PID: $TARGET_PID"
echo "Run: kill $LAUNCH_PID $TARGET_PID  # to clean up"
