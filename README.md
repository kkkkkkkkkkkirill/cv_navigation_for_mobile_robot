# amr_stage4_cv_nav

Computer-vision navigation layer for an autonomous warehouse mobile robot — built on **ROS 2 Jazzy** + **Gazebo Harmonic** and integrated with a Nav2 baseline.

This repo is the latest iteration (Stage 4) of a bachelor's thesis project at HSE FCS. It adds a complementary CV perception stack on top of an existing LiDAR-based Nav2 navigation system. The CV layer is drop-in compatible — it uses the same standard ROS 2 messages and the same operator UI as the LiDAR baseline.

---

## What's inside

| Component | Description |
| --- | --- |
| `ros2_ws/` | ROS 2 package `kolestel_rover_description` — robot URDF, Gazebo worlds, launch files, Nav2 config, ArUco models, navigation map. |
| `web_app/` | FastAPI backend + PWA frontend + thin rclpy bridges (status, dispatch, Nav2 task). |
| `shared/` | Canonical warehouse layout (`user_saved_layout.sdf`), aligned 2D map (`warehouse_map.yaml`), ArUco world poses. |
| `*.sh` (root) | One-command run scripts for the various launch profiles. |
| `README_*.md` | Per-stage notes — Stage 1 → Stage 4 evolution and ArUco update log. |

---

## CV layer — what it does

Three perception channels feeding a single graph planner (`cv_navigator`):

1. **Stick-to-Line** (`line_follower` node) — HSV thresholding + morphological erosion + centroid extraction + P-controller. Follows painted yellow aisles at 30 FPS on plain CPU. No neural network.
2. **ArUco localisation** (`aruco_node`) — detection + `solvePnP` (with `SOLVEPNP_IPPE_SQUARE`) on 30 markers placed on shelf faces. Provides absolute `odom → map` TF corrections to fix wheel-odometry drift.
3. **Depth-camera fusion** — Intel RealSense D435 point cloud projected into the LiDAR costmap as inflation cells. Catches small floor obstacles that fall between LiDAR beams.

Architecture diagram and full algorithmic breakdown are in the bachelor's thesis defence deck (not in this repo).

---

## Quick start (simulation)

```bash
# clean any old Gazebo state
pkill -9 -f gz; pkill -9 -f ros; pkill -9 -f rviz; pkill -9 -f uvicorn
rm -rf ~/.gz/sim ~/.gz/fuel ~/.gazebo /tmp/.gazebo* /tmp/gz_*
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log

# build the ROS 2 package
bash build_ros2.sh

# run the full CV-nav stack (4 terminals)
bash run_gazebo_cv_nav.sh
bash web_app/app/backend/run_external_backend.sh
bash web_app/app/ros2_bridge/run_robot_status_bridge.sh
bash web_app/app/ros2_bridge/run_task_nav2_bridge.sh
```

Then open `http://127.0.0.1:8010` in your browser (hard-reload with **Ctrl+Shift+R**).

For variants — Gazebo only, Nav2-only baseline, smoke tests — see the other `run_*.sh` scripts at the repo root.

---

## Stage history

| Stage | Focus | README |
| --- | --- | --- |
| 1 | Canonical user world + synchronised 2D map | [`README_STAGE1.md`](README_STAGE1.md) |
| 3 | Dispatch / event bridge contract | [`README_STAGE3.md`](README_STAGE3.md) |
| 4 | Real Nav2 integration | [`README_STAGE4.md`](README_STAGE4.md) |
| 4-cv | CV layer + 30 ArUco markers + depth fusion (this version) | [`README_ARUCO_UPDATE.md`](README_ARUCO_UPDATE.md) |

Notes on hard-fix workarounds and Gazebo cache pitfalls live in `README_HARD_FIX_NOTES.txt` and `README_WORLD_CACHE_NOTE.txt`.

---

## Tech stack

- **Middleware:** ROS 2 Jazzy
- **Simulator:** Gazebo Harmonic
- **CV:** OpenCV 4.x (Python, no GPU)
- **Sensors (sim):** Intel RealSense D435, Livox MID-360 LiDAR
- **Navigation:** Nav2 baseline + custom `cv_navigator` graph planner
- **Backend:** FastAPI + SQLite task queue + WebSocket
- **Frontend:** PWA (vanilla HTML/JS)

---

## Hardware target

- Simulation: any Linux box with ROS 2 Jazzy + Gazebo Harmonic
- On-board mini-PC: NucBox K10
- Field deployment (separate workstream): Raspberry Pi 5 with IMX219-160 IR-CUT camera

See [`MINIPC_RUN.md`](MINIPC_RUN.md) for mini-PC deployment notes.

---

## Author

Kirill Budyak — HSE Faculty of Computer Science, DSBA programme, group 221, year 4.
Supervisor: Ivan Stanislavovich Kopylov (Senior Lecturer, Big Data & Information Retrieval).

Bachelor's thesis · Pre-defence 2026.
