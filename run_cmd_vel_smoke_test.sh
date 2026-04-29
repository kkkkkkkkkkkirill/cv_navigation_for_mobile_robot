#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$ROOT_DIR/ros2_ws"

if [ ! -f "$ROS_WS/install/setup.bash" ]; then
  echo "ROS workspace is not built: $ROS_WS/install/setup.bash" >&2
  echo "Run: $ROOT_DIR/build_ros2.sh" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

echo "[1/3] Forward cmd_vel for 3 seconds"
timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.20}, angular: {z: 0.0}}" >/dev/null || true
sleep 1
echo "[2/3] Rotate cmd_vel for 2 seconds"
timeout 2 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.40}}" >/dev/null || true
sleep 1
echo "[3/3] Stop"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
