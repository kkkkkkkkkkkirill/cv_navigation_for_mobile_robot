import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_map_file(pkg_dir: str) -> str:
    env_path = os.environ.get('AMR_WAREHOUSE_MAP_PATH')
    if env_path and os.path.isfile(env_path):
        return env_path
    return os.path.join(pkg_dir, 'config', 'warehouse_map.yaml')


def _resolve_world_file(pkg_dir: str) -> str:
    env_path = os.environ.get('AMR_GAZEBO_WORLD_PATH')
    if env_path and os.path.isfile(env_path):
        return env_path
    return os.path.join(pkg_dir, 'worlds', 'user_saved_layout.sdf')


def _load_depot_pose(map_file: str) -> tuple[str, str]:
    with open(map_file, encoding='utf-8') as f:
        data = yaml.safe_load(f)['warehouse']
    depot = data['stations']['depot']
    return str(depot['x']), str(depot['y'])


def generate_launch_description():
    pkg = get_package_share_directory('kolestel_rover_description')
    models_path = os.path.join(pkg, 'models')
    worlds_path = os.path.join(pkg, 'worlds')
    xacro_file = os.path.join(pkg, 'urdf', 'kolestel_rover.urdf.xacro')
    bridge_cfg = os.path.join(pkg, 'config', 'ros_gz_bridge.yaml')
    world_file = _resolve_world_file(pkg)
    map_file = _resolve_map_file(pkg)
    depot_x, depot_y = _load_depot_pose(map_file)
    depot_x_f = float(depot_x)
    depot_y_f = float(depot_y)

    share_root = os.path.dirname(pkg)
    resource_path = ':'.join(filter(None, [
        share_root,
        models_path,
        worlds_path,
        pkg,
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    enable_aux_nodes = LaunchConfiguration('enable_aux_nodes')
    publish_static_map_to_odom = LaunchConfiguration('publish_static_map_to_odom')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [world, ' -r -v 3']}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'kolestel_rover',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}],
    )

    # CV nav uses aruco_node as the map->odom publisher (drift correction),
    # so for that stack this publisher must be disabled.
    # Note: we use a 30Hz Python republisher instead of static_transform_publisher
    # because static_transform_publisher uses TRANSIENT_LOCAL QoS, which is
    # sometimes missed by late subscribers (notably RViz on slow startup).
    # The continuous /tf publish guarantees every subscriber sees the chain.
    map_to_odom_tf = Node(
        package='kolestel_rover_description',
        executable='map_to_odom_publisher.py',
        name='map_to_odom_publisher',
        output='screen',
        parameters=[{
            'x': depot_x_f,
            'y': depot_y_f,
            'z': 0.0,
            'yaw': 0.0,
            'parent_frame': 'map',
            'child_frame': 'odom',
            'rate_hz': 30.0,
            'use_sim_time': True,
        }],
        condition=IfCondition(publish_static_map_to_odom),
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'kolestel_rover/base_footprint/imu_sensor', 'imu_link'],
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_link', 'kolestel_rover/base_footprint/gpu_lidar'],
    )

    # Gazebo Harmonic publishes camera frames in BODY convention (X-forward,
    # Y-left, Z-up) — same as lidar — NOT optical (X-right, Y-down, Z-forward).
    # So we need a body->optical rotation between the URDF's *_optical_frame
    # (REP-103) and the Gazebo sensor frame. RPY positional form is yaw pitch
    # roll, so yaw=pi/2 pitch=-pi/2 roll=0 gives the required body->optical mapping.
    camera_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_tf',
        arguments=['0', '0', '0', '1.5708', '-1.5708', '0',
                   'camera_depth_optical_frame', 'kolestel_rover/base_footprint/depth_camera'],
    )

    camera_color_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_tf',
        arguments=['0', '0', '0', '1.5708', '-1.5708', '0',
                   'camera_color_optical_frame', 'kolestel_rover/base_footprint/rgb_camera'],
    )

    odom_tf = Node(
        package='kolestel_rover_description',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    converter = Node(
        package='kolestel_rover_description',
        executable='pointcloud_to_livox.py',
        name='pointcloud_to_livox',
        output='screen',
        parameters=[{
            'input_topic': '/lidar',
            'output_topic': '/livox/lidar',
            'scan_rate': 8.0,
            'use_sim_time': True,
        }],
        condition=IfCondition(enable_aux_nodes),
    )

    warehouse_visualizer = Node(
        package='kolestel_rover_description',
        executable='warehouse_map_visualizer.py',
        name='warehouse_map_visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(enable_aux_nodes),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('spawn_x', default_value=depot_x),
        DeclareLaunchArgument('spawn_y', default_value=depot_y),
        DeclareLaunchArgument('spawn_z', default_value='0.10'),
        DeclareLaunchArgument('enable_aux_nodes', default_value='false'),
        DeclareLaunchArgument('publish_static_map_to_odom', default_value='true'),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
        rsp,
        gz_sim,
        map_to_odom_tf,
        imu_tf,
        lidar_tf,
        camera_depth_tf,
        camera_color_tf,
        TimerAction(period=8.0, actions=[spawn]),
        TimerAction(period=10.0, actions=[bridge]),
        TimerAction(period=12.0, actions=[odom_tf, converter, warehouse_visualizer]),
    ])
