# Stage 3 — dispatch/event bridge

This stage adds the backend contract for external robot control:

- `GET /robot/dispatch`
- `POST /robot/event`

and a dry-run ROS2 bridge:

- `robot_status_bridge.py` keeps live pose flowing from Gazebo `/odom` to backend
- `task_command_bridge.py` polls dispatch and simulates arrival events

## Launch order

### 1) Gazebo / ROS2
```bash
cd ~/ros2_ws/src
rm -rf kolestel_rover_description
cp -r ~/Desktop/amr_stage3_dispatch_bridge/amr_stage2_precise_web_map/ros2_ws/src/kolestel_rover_description .

cd ~/ros2_ws
export AMR_WAREHOUSE_MAP_PATH=~/Desktop/amr_stage3_dispatch_bridge/amr_stage2_precise_web_map/shared/warehouse_map.yaml
rm -rf build/kolestel_rover_description install/kolestel_rover_description log
colcon build --packages-select kolestel_rover_description
source install/setup.bash
ros2 launch kolestel_rover_description gazebo.launch.py
```

### 2) Backend
```bash
cd ~/Desktop/amr_stage3_dispatch_bridge/amr_stage2_precise_web_map/web_app/app/backend
source .venv/bin/activate
export AMR_USE_INTERNAL_SIMULATOR=false
export AMR_WAREHOUSE_MAP_PATH=~/Desktop/amr_stage3_dispatch_bridge/amr_stage2_precise_web_map/shared/warehouse_map.yaml
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

### 3) Live robot status bridge
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export AMR_BACKEND_URL=http://127.0.0.1:8000
python3 ~/Desktop/amr_stage3_dispatch_bridge/amr_stage2_precise_web_map/web_app/app/ros2_bridge/robot_status_bridge.py
```

### 4) Dispatch / event bridge
```bash
cd ~/Desktop/amr_stage3_dispatch_bridge/amr_stage2_precise_web_map/web_app/app/ros2_bridge
export AMR_BACKEND_URL=http://127.0.0.1:8000
./run_task_command_bridge.sh
```

## Demo scenario

1. Open web UI
2. Create a task, e.g. `shelf_a1 -> shelf_b1`
3. `GET /robot/dispatch` assigns the pending task to the robot
4. `task_command_bridge.py` emits:
   - `nav_started`
   - `arrived_pickup`
5. Confirm load in web UI
6. Bridge emits:
   - `nav_started`
   - `arrived_dropoff`
7. Confirm unload in web UI
8. Bridge emits:
   - `nav_started` for return
   - `returned_to_depot`

No Nav2 yet in this stage; motion is a dry-run task lifecycle bridge.
