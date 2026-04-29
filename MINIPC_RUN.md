# Mini PC Runbook

Run everything from:

```bash
cd /home/rover/Desktop/amr_stage4_nav2
```

## Install Runtime Packages

```bash
sudo apt update
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-collision-monitor ros-jazzy-nav2-velocity-smoother python3-colcon-common-extensions python3-catkin-pkg
```

## Build ROS 2 Package

```bash
./build_ros2.sh
```

## Clean Old Runtime State

```bash
./reset_runtime.sh
```

## Terminal 1: Gazebo + Nav2

```bash
./run_gazebo_nav2.sh
```

## Terminal 2: Backend

```bash
./web_app/app/backend/run_external_backend.sh
```

## Terminal 3: Robot Status Bridge

```bash
./web_app/app/ros2_bridge/run_robot_status_bridge.sh
```

## Terminal 4: Task to Nav2 Bridge

```bash
./web_app/app/ros2_bridge/run_task_nav2_bridge.sh
```

Open:

```text
http://127.0.0.1:8010
```

## Quick Checks

```bash
source /opt/ros/jazzy/setup.bash
source /home/rover/Desktop/amr_stage4_nav2/ros2_ws/install/setup.bash
ros2 topic echo /odom --once
ros2 topic hz /lidar
ros2 topic echo /cmd_vel --once
curl http://127.0.0.1:8010/robot/status
```

