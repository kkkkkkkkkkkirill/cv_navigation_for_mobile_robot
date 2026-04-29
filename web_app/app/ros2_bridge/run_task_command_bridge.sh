#!/usr/bin/env bash
set -eo pipefail
unset http_proxy HTTP_PROXY https_proxy HTTPS_PROXY all_proxy ALL_PROXY
export NO_PROXY=127.0.0.1,localhost
export no_proxy=127.0.0.1,localhost
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ROS_WS="$ROOT_DIR/ros2_ws"
if [ ! -f "$ROS_WS/install/setup.bash" ]; then
  echo "ROS workspace is not built: $ROS_WS/install/setup.bash" >&2
  echo "Run: cd $ROS_WS && colcon build --packages-select kolestel_rover_description" >&2
  exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
export AMR_BACKEND_URL="${AMR_BACKEND_URL:-http://127.0.0.1:8010}"
export AMR_AUTO_SIMULATE_ARRIVAL="${AMR_AUTO_SIMULATE_ARRIVAL:-true}"
export AMR_SIMULATED_TRAVEL_TIME_S="${AMR_SIMULATED_TRAVEL_TIME_S:-4.0}"
python3 "$SCRIPT_DIR/task_command_bridge.py"
