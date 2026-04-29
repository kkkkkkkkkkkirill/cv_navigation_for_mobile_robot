This package uses user_saved_layout.sdf as the active Gazebo world.
There is no warehouse.sdf in the current project tree.

Extra safeguards added:
- build_ros2.sh always deletes ros2_ws/build, ros2_ws/install, ros2_ws/log
- build_ros2.sh checks source and installed user_saved_layout.sdf for the bad token 41.632-0.191335
- run_gazebo_only.sh and run_gazebo_nav2.sh force AMR_GAZEBO_WORLD_PATH to the installed user_saved_layout.sdf
- clean_rebuild_world_safe.sh kills ROS/Gazebo and does a fully clean rebuild
