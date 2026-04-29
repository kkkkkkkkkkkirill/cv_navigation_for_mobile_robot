#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$ROOT_DIR/ros2_ws"
SRC_WORLD="$ROS_WS/src/kolestel_rover_description/worlds/user_saved_layout.sdf"
INSTALL_WORLD="$ROS_WS/install/kolestel_rover_description/share/kolestel_rover_description/worlds/user_saved_layout.sdf"

deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH CMAKE_PREFIX_PATH

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "[build_ros2] Cleaning colcon caches..."
rm -rf "$ROS_WS/build" "$ROS_WS/install" "$ROS_WS/log"

echo "[build_ros2] Removing stale world files from workspace (if any)..."
find "$ROOT_DIR" -type f \( -name 'warehouse.sdf' -o -name 'dynamic_warehouse_harmonic*.sdf' \) -print || true

if [ -f "$SRC_WORLD" ]; then
  if grep -q '41.632-0.191335' "$SRC_WORLD"; then
    echo "[build_ros2] ERROR: bad pose token found in source world: $SRC_WORLD" >&2
    exit 1
  fi
fi

cd "$ROS_WS"
colcon build --packages-select kolestel_rover_description

if [ -f "$INSTALL_WORLD" ]; then
  if grep -q '41.632-0.191335' "$INSTALL_WORLD"; then
    echo "[build_ros2] ERROR: bad pose token found in installed world: $INSTALL_WORLD" >&2
    exit 1
  fi
fi

echo "[build_ros2] Done."
