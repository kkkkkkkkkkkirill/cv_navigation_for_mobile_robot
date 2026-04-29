#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$ROOT_DIR/ros2_ws"

echo "[clean_rebuild_world_safe] Killing ROS / Gazebo..."
pkill -f ros2 || true
pkill -f gz || true
pkill -f gazebo || true
pkill -f rviz || true

echo "[clean_rebuild_world_safe] Removing build/install/log and stale runtime caches..."
rm -rf "$ROS_WS/build" "$ROS_WS/install" "$ROS_WS/log"
rm -rf "$HOME/.ros/log"/* 2>/dev/null || true

bash "$ROOT_DIR/build_ros2.sh"
