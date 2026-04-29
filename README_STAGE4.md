# Stage 4 - Real Nav2 Integration

This stage is built strictly on top of the provided `amr_stage3_dispatch_bridge_fix` project.

## What was added
- Nav2 occupancy map generated from the current shared warehouse geometry
- Nav2 parameter file
- `nav2.launch.py`
- `sim_nav2.launch.py` to start Gazebo first and Nav2 after startup
- proxy-safe localhost bridge scripts
- `task_nav2_bridge.py` for real NavigateToPose execution
- helper script `generate_nav2_map.py`

## New files
- `ros2_ws/src/kolestel_rover_description/maps/warehouse_nav2_map.pgm`
- `ros2_ws/src/kolestel_rover_description/maps/warehouse_nav2_map.yaml`
- `ros2_ws/src/kolestel_rover_description/config/nav2_params.yaml`
- `ros2_ws/src/kolestel_rover_description/launch/nav2.launch.py`
- `ros2_ws/src/kolestel_rover_description/launch/sim_nav2.launch.py`
- `ros2_ws/src/kolestel_rover_description/scripts/generate_nav2_map.py`
- `web_app/app/ros2_bridge/task_nav2_bridge.py`
- `web_app/app/ros2_bridge/run_task_nav2_bridge.sh`

## Important notes
This is the first real navigation stage. It intentionally keeps:
- the current shared map / station contract
- the current backend API
- the current `robot_status_bridge.py` based on `odom + map origin`

It does not yet include:
- AMCL
- dynamic costmap obstacle layers from LiDAR
- ArUco docking refinement
- automatic line-follow arbitration

## Bringup order

### 1. Build ROS2 package
```bash
cd ~/ros2_ws/src
rm -rf kolestel_rover_description
cp -r ~/Desktop/amr_stage4_nav2/ros2_ws/src/kolestel_rover_description .

cd ~/ros2_ws
export AMR_WAREHOUSE_MAP_PATH=~/Desktop/amr_stage4_nav2/shared/warehouse_map.yaml
rm -rf build/kolestel_rover_description install/kolestel_rover_description log
colcon build --packages-select kolestel_rover_description
source install/setup.bash
```

### 2. Start Gazebo + Nav2
```bash
ros2 launch kolestel_rover_description sim_nav2.launch.py
```

### 3. Start backend
```bash
cd ~/Desktop/amr_stage4_nav2/web_app/app/backend
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
rm -f amr.db
export AMR_USE_INTERNAL_SIMULATOR=false
export AMR_WAREHOUSE_MAP_PATH=~/Desktop/amr_stage4_nav2/shared/warehouse_map.yaml
uvicorn app.main:app --host 0.0.0.0 --port 8010
```

### 4. Start status bridge
```bash
cd ~/Desktop/amr_stage4_nav2/web_app/app/ros2_bridge
export AMR_BACKEND_URL=http://127.0.0.1:8010
export AMR_MAP_ORIGIN_X=10.5
export AMR_MAP_ORIGIN_Y=-8.0
./run_robot_status_bridge.sh
```

### 5. Start real Nav2 task bridge
```bash
cd ~/Desktop/amr_stage4_nav2/web_app/app/ros2_bridge
export AMR_BACKEND_URL=http://127.0.0.1:8010
export AMR_WAREHOUSE_MAP_PATH=~/Desktop/amr_stage4_nav2/shared/warehouse_map.yaml
./run_task_nav2_bridge.sh
```

### 6. Open web
- `http://127.0.0.1:8010`

## Expected behavior
- create a task in web
- backend exposes dispatch
- `task_nav2_bridge.py` sends `NavigateToPose`
- robot physically moves in Gazebo
- on Nav2 success backend receives:
  - `arrived_pickup`
  - `arrived_dropoff`
  - `returned_to_depot`

## Fallback
If you want the old dry-run bridge, keep using:
```bash
./run_task_command_bridge.sh
```
