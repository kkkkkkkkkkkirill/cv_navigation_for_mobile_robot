# CV Navigation for Mobile Robot

Computer-vision navigation stack for a four-wheel warehouse mobile robot. The CV stack sits alongside a LiDAR + Nav2 baseline and adds three things the lidar can't do on its own: it follows painted floor lines, it corrects odometry drift against ArUco markers on shelf faces, and it adds low floor objects to the costmap using a depth camera. A YOLOv8 detector publishes class labels for the operator UI.

Everything is validated in a Gazebo Harmonic simulation of a small warehouse. The line-following subsystem also runs on a physical chassis with a USB camera (outdoor course on yellow tape).

This is the codebase for my bachelor's thesis at HSE FCS, DSBA programme, 2026.

## Layout

- `ros2_ws/` — ROS 2 package: URDF, Gazebo worlds, Nav2 config, launch files, the CV nodes.
- `web_app/` — FastAPI backend, PWA frontend, and three small rclpy bridges (status, dispatch, Nav2 task).
- `shared/` — warehouse SDF, the 2D map aligned to it, ArUco world poses.
- `real_robot_demo/` — standalone runner for the physical-robot line-follow test (no ROS, no Gazebo).
- `run_*.sh` — one-command launch scripts at the repo root.

## CV nodes

**Line follower.** HSV mask of the yellow lane, morphological cleanup, centroid extraction, P-controller on the cross-track error. Pure OpenCV, 30 FPS on CPU.

**ArUco localizer.** `cv2.aruco.detectMarkers` plus `solvePnP` with `SOLVEPNP_IPPE_SQUARE` on 30 markers fixed to shelf faces. The result is fused with wheel odometry through a predict-update planar pose filter; outliers are rejected and the filter snaps back to the marker pose after divergence.

**Depth obstacle layer.** Intel RealSense D435 point cloud, projected into the Nav2 costmap as inflation cells. The use case is small floor objects (the canonical test is a 0.4 × 0.4 × 0.3 m cardboard box) that fall under the Livox Mid-360 scan plane.

**YOLOv8 detector.** `yolov8n-oiv7.pt` from Ultralytics, 601 Open Images classes, CPU inference. The class name flows into the operator-UI obstacle banner via WebSocket.

**Graph planner.** Replaces Nav2's grid-based global planner with a route over the warehouse station graph (23 stations, 31 nodes, 51 lanes). Exposes the same `nav2_msgs/action/NavigateToPose` interface, so the operator UI drives either stack unchanged.

## Run it in simulation

```bash
# clear stale Gazebo cache and previous build
pkill -9 -f gz; pkill -9 -f ros; pkill -9 -f rviz; pkill -9 -f uvicorn
rm -rf ~/.gz/sim ~/.gz/fuel ~/.gazebo /tmp/.gazebo* /tmp/gz_*
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log

# build
bash build_ros2.sh

# four terminals
bash run_gazebo_cv_nav.sh
bash web_app/app/backend/run_external_backend.sh
bash web_app/app/ros2_bridge/run_robot_status_bridge.sh
bash web_app/app/ros2_bridge/run_task_nav2_bridge.sh
```

Open `http://127.0.0.1:8010` in the browser, hard-reload with Ctrl+Shift+R.

Other entry points: `run_gazebo_only.sh` for the simulator alone, `run_nav2_baseline.sh` for the LiDAR-only stack to compare against.

## Real-robot demo

`real_robot_demo/` runs the line-follow logic on a DEXP DWC-FHD03 USB webcam, no ROS, no Gazebo:

```bash
cd real_robot_demo
python3 calibrate.py    # one-time, chessboard intrinsics
python3 line_follow.py --camera 0
```

The course is yellow tape on asphalt.

## Stack

ROS 2 Jazzy, Gazebo Harmonic, OpenCV 4 (no GPU), Nav2 baseline, FastAPI + WebSocket, vanilla PWA on the frontend. Sensors in sim: Livox Mid-360 LiDAR and Intel RealSense D435; on hardware so far: DEXP USB webcam.
