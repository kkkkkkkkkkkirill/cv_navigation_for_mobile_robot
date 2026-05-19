cmake_minimum_required(VERSION 3.8)
project(kolestel_rover_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  worlds
  launch
  config
  maps
  models
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/pointcloud_to_livox.py
  scripts/odom_to_tf.py
  scripts/warehouse_map_visualizer.py
  scripts/generate_nav2_map.py
  scripts/line_follower_node.py
  scripts/cv_navigator.py
  scripts/aruco_node.py
  scripts/yolo_node.py
  scripts/depth_cloud_republisher.py
  scripts/map_to_odom_publisher.py
  scripts/lidar_map_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
