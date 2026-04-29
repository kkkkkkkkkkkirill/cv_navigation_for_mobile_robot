#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$ROOT_DIR/ros2_ws"
ACTIVE_WORLD="$ROS_WS/install/kolestel_rover_description/share/kolestel_rover_description/worlds/user_saved_layout.sdf"

if [ ! -f "$ROS_WS/install/setup.bash" ]; then
  echo "ROS workspace is not built: $ROS_WS/install/setup.bash" >&2
  echo "Run: $ROOT_DIR/build_ros2.sh" >&2
  exit 1
fi

if [ ! -f "$ACTIVE_WORLD" ]; then
  echo "Active world file not found: $ACTIVE_WORLD" >&2
  echo "Run: $ROOT_DIR/build_ros2.sh" >&2
  exit 1
fi

if grep -q '41.632-0.191335' "$ACTIVE_WORLD"; then
  echo "Bad cached world detected in install tree: $ACTIVE_WORLD" >&2
  echo "Run: $ROOT_DIR/clean_rebuild_world_safe.sh" >&2
  exit 1
fi

echo "[run_gazebo_cv_nav] Using world: $ACTIVE_WORLD"
echo "[run_gazebo_cv_nav] Navigation: CV (stick-to-line), NOT Nav2"

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

export AMR_WAREHOUSE_MAP_PATH="$ROOT_DIR/shared/warehouse_map.yaml"
export AMR_GAZEBO_WORLD_PATH="$ACTIVE_WORLD"
ros2 launch kolestel_rover_description sim_cv_nav.launch.py
