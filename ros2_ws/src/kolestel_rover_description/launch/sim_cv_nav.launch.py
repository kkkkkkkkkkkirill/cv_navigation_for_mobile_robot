import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _resolve_map_file(pkg_dir: str) -> str:
    env_path = os.environ.get('AMR_WAREHOUSE_MAP_PATH')
    if env_path and os.path.isfile(env_path):
        return env_path
    return os.path.join(pkg_dir, 'config', 'warehouse_map.yaml')


def _load_depot_xy(map_file: str) -> tuple[float, float]:
    with open(map_file, encoding='utf-8') as f:
        data = yaml.safe_load(f)['warehouse']
    depot = data['stations']['depot']
    return float(depot['x']), float(depot['y'])


def generate_launch_description():
    pkg = get_package_share_directory('kolestel_rover_description')
    map_file = _resolve_map_file(pkg)
    depot_x, depot_y = _load_depot_xy(map_file)
    marker_world_poses_path = os.path.join(pkg, 'config', 'aruco_world_poses.yaml')

    # CV stack: aruco_node owns the map->odom transform (drift correction).
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'publish_static_map_to_odom': 'false'}.items(),
    )

    line_follower = Node(
        package='kolestel_rover_description',
        executable='line_follower_node.py',
        name='line_follower',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image',
            'cmd_vel_topic': '/line_follower/cmd_vel',
            'linear_speed': 0.4,
            'kp': 0.003,
            'scan_y1': 380,
            'scan_y2': 430,
            'intersection_threshold': 250,
            'line_lost_threshold': 25,
        }],
    )

    cv_navigator = Node(
        package='kolestel_rover_description',
        executable='cv_navigator.py',
        name='cv_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    depth_cloud_republisher = Node(
        package='kolestel_rover_description',
        executable='depth_cloud_republisher.py',
        name='depth_cloud_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/camera/depth/points',
            'output_topic': '/camera/depth/points_base',
            'target_frame': 'base_footprint',
        }],
    )

    yolo_node = Node(
        package='kolestel_rover_description',
        executable='yolo_node.py',
        name='yolo_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image',
            'debug_topic': '/yolo/debug_image',
            'model_path': 'yolov8n-oiv7.pt',
            'confidence_threshold': 0.15,
            'iou_threshold': 0.45,
            'downscale': 1.0,
        }],
    )

    aruco_node = Node(
        package='kolestel_rover_description',
        executable='aruco_node.py',
        name='aruco_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'marker_side_m': 0.40,
            'image_topic': '/camera/image',
            'camera_info_topic': '/camera/camera_info',
            'debug_topic': '/aruco/debug_image',
            'dictionary': 'DICT_6X6_250',
            # frames
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'camera_optical_frame': 'camera_color_optical_frame',
            # pose fuser (new architecture)
            'enable_drift_correction': True,
            'marker_world_poses_path': marker_world_poses_path,
            'initial_pose_x': depot_x,
            'initial_pose_y': depot_y,
            'initial_pose_yaw': 0.0,
            'filter_alpha_xy': 0.10,
            'filter_alpha_yaw': 0.10,
            'min_marker_distance_m': 0.8,
            'max_marker_distance_m': 2.5,
            'max_marker_obliqueness_deg': 45.0,
            'measurement_outlier_xy_m': 0.7,
            'measurement_outlier_yaw_rad': 0.5,
            'consecutive_outliers_for_resync': 5,
            'tf_publish_rate_hz': 50.0,
        }],
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(period=10.0, actions=[line_follower]),
        TimerAction(period=11.0, actions=[aruco_node]),
        TimerAction(period=12.0, actions=[cv_navigator]),
        TimerAction(period=13.0, actions=[yolo_node, depth_cloud_republisher]),
    ])
