# Changelog

## CV-stack update (after thesis defence prep)

### ArUco-based localization (completed)
- `aruco_node.py` rewritten as a planar pose fuser:
  - **Predict** step at 50 Hz from `/odom` deltas applied to `T_WB` in the `map` frame.
  - **Update** step at ~30 Hz: low-pass blend of the SolvePnP-derived T_WB toward the filter state (`alpha_xy = 0.10`, `alpha_yaw = 0.10`).
  - **Outlier rejection** vs filter (`>0.7m` or `>29°`) plus **divergence recovery** after 5 consecutive rejected measurements.
  - **Min distance gate** at 0.8 m and **obliqueness gate** at 45° to filter low-quality solvePnP returns.
  - Publishes `/aruco/pose` (PoseStamped) and `map → odom` TF correction.
- `cv_navigator.py` consumes `/aruco/pose` directly (single source of truth). Controller and UI now read the same fused pose.
- 30 ArUco markers defined in `config/aruco_world_poses.yaml`.

### Depth-camera obstacle layer (completed)
- New `depth_cloud_republisher.py`: subscribes to `/camera/depth/points` (Gazebo body-convention BODY frame in this build), transforms to `base_footprint` (X-forward, Y-left, Z-up) and republishes as `/camera/depth/points_base`.
- `cv_navigator.py` adds a `depth_cb` running on `/camera/depth/points` with body-convention filtering. The depth layer detects **low floor obstacles the Mid-360 lidar cannot see** (lidar at z = 0.76 m has min vertical -7°, blind to objects shorter than ~0.64 m at 1 m forward).
- Test case in `worlds/user_saved_layout.sdf`: a single 0.4 × 0.4 × 0.3 m **floor box** on the depot → SHELF_E1 path. Lidar passes over, depth camera triggers the stop.

### YOLO detection (completed)
- New `yolo_node.py`: subscribes to `/camera/image`, runs `yolov8n-oiv7.pt` (601 Open Images classes — includes `Box`, `Cardboard box`, `Person`, `Pallet`, `Trash can`, `Bucket`, `Shelf`) on CPU, publishes `/yolo/debug_image` and `/yolo/detections` (`std_msgs/String`, comma-separated class list).
- `cv_navigator.py` includes the last YOLO `detected_class` in the obstacle POST so the UI can show the recognised class.

### Web UI obstacle warning (completed)
- New backend endpoint: `POST /robot/obstacle` accepts `{blocked, source, distance_m, detected_class}` and broadcasts to all WS clients.
- PWA renders a fixed-top **red banner** while obstacle_blocked is true, including YOLO class label.
- `robot_status_bridge.py` reads pose via TF `map → base_footprint` (so the icon reflects the fused pose, not raw odom).

### Lidar visualisation TF fix (completed)
- Replaced one-shot static `map → odom` publisher with `map_to_odom_publisher.py` that publishes on `/tf` at 30 Hz (works around RViz / DDS subscribing late to TRANSIENT_LOCAL latched messages).
- New `lidar_map_republisher.py` transforms `/lidar` (body frame) to `/lidar_map` (map frame) so RViz never has to do a TF lookup — eliminates the "lidar rotates with robot" artefact during fast rotation.
- Static TFs for the camera (`camera_*_optical_frame`) now include the correct body→optical rotation `(yaw=π/2, pitch=-π/2)` because Gazebo Harmonic's `gpu_camera` publishes in BODY convention.

### Real-robot stick-to-line demo (new)
- `real_robot_demo/` — standalone YOLO11n + OC-SORT person tracker exercised on the DEXP DWC-FHD03 USB camera, with the same detection/tracking logic that the simulated stack uses. No ROS, no Gazebo — just OpenCV + a window.

## Files added

```
ros2_ws/src/kolestel_rover_description/scripts/
  yolo_node.py
  depth_cloud_republisher.py
  map_to_odom_publisher.py
  lidar_map_republisher.py

ros2_ws/src/kolestel_rover_description/config/
  aruco_world_poses.yaml
  cv_nav.rviz

ros2_ws/src/kolestel_rover_description/models/
  aruco_marker_0/ … aruco_marker_29/  (30 markers)

real_robot_demo/
  run.sh
  track_dexp.py
```

## Files significantly changed

```
ros2_ws/src/kolestel_rover_description/scripts/aruco_node.py        (pose fuser)
ros2_ws/src/kolestel_rover_description/scripts/cv_navigator.py      (fused-pose source, obstacle layers, YOLO event)
ros2_ws/src/kolestel_rover_description/launch/gazebo.launch.py      (map_to_odom publisher, camera TF rotation)
ros2_ws/src/kolestel_rover_description/launch/sim_cv_nav.launch.py  (fuser params, YOLO, depth republisher)
ros2_ws/src/kolestel_rover_description/CMakeLists.txt               (new scripts)
ros2_ws/src/kolestel_rover_description/urdf/kolestel_rover.urdf.xacro (camera far clip 10 → 100 m)
ros2_ws/src/kolestel_rover_description/worlds/user_saved_layout.sdf  (30 ArUco markers + floor box)
web_app/app/backend/app/routers/robot.py                            (/robot/obstacle endpoint)
web_app/app/pwa/index.html                                          (obstacle banner)
web_app/app/ros2_bridge/robot_status_bridge.py                      (TF lookup)
```
