# Stage 1 — User world only + synchronized map

This stage uses **only your saved Gazebo world** as the canonical warehouse scene.

## Canonical files

- `shared/user_saved_layout.sdf` — your canonical warehouse world.
- `shared/warehouse_map.yaml` — the canonical 2D / backend / RViz map aligned to that world.
- `ros2_ws/src/kolestel_rover_description/worlds/user_saved_layout.sdf` — same world copied into the ROS2 package.
- `ros2_ws/src/kolestel_rover_description/config/warehouse_map.yaml` — same map copied into the ROS2 package.

There is no fallback to the old package warehouse layout in this stage.  
Gazebo starts `user_saved_layout.sdf`, and the backend / web use the same station and graph coordinates from `warehouse_map.yaml`.

## Coordinate system used

Aisles:
- A = `x=2`
- B = `x=9`
- C = `x=15`
- D = `x=22`
- E = `x=30`

Rows:
- south = `y=-8`
- row1 = `y=-2`
- row2 = `y=10`
- row3 = `y=22`
- row4 = `y=35`

Depot:
- `x=6`
- `y=-11`

## Run ROS2

```bash
cd ~/ros2_ws/src
rm -rf kolestel_rover_description
cp -r /PATH/TO/amr_stage1_user_world_synced/ros2_ws/src/kolestel_rover_description .

cd ~/ros2_ws
export AMR_WAREHOUSE_MAP_PATH=/PATH/TO/amr_stage1_user_world_synced/shared/warehouse_map.yaml
rm -rf build/kolestel_rover_description install/kolestel_rover_description log
colcon build --packages-select kolestel_rover_description
source install/setup.bash
ros2 launch kolestel_rover_description gazebo.launch.py
```

## Run backend

```bash
cd /PATH/TO/amr_stage1_user_world_synced/web_app/app/backend
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
export AMR_USE_INTERNAL_SIMULATOR=false
export AMR_WAREHOUSE_MAP_PATH=/PATH/TO/amr_stage1_user_world_synced/shared/warehouse_map.yaml
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

## Notes

- Gazebo and ROS2 use `user_saved_layout.sdf`.
- Backend and web use the synchronized `warehouse_map.yaml`.
- The web app does not render the `.sdf` directly; instead it uses the same station / node / lane coordinates extracted for this world.


## Precise map alignment

The shared warehouse map has been aligned to the user-specified aisle / row coordinate system:
- aisles: x = 2, 9, 15, 22, 30
- cross aisles: y = -8, -2, 10, 22, 35
- depot: x = 6, y = -11
- shelf approach targets are derived from this same frame.
