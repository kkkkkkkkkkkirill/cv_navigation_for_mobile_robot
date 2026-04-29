import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _resolve_map_file(pkg_dir: str) -> str:
    env_path = os.environ.get('AMR_NAV2_MAP_PATH')
    if env_path and os.path.isfile(env_path):
        return env_path
    return os.path.join(pkg_dir, 'maps', 'warehouse_nav2_map.yaml')


def _resolve_params_file(pkg_dir: str) -> str:
    env_path = os.environ.get('AMR_NAV2_PARAMS_PATH')
    if env_path and os.path.isfile(env_path):
        return env_path
    return os.path.join(pkg_dir, 'config', 'nav2_params.yaml')


def generate_launch_description():
    pkg = get_package_share_directory('kolestel_rover_description')
    map_file = _resolve_map_file(pkg)
    params_file = _resolve_params_file(pkg)

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_file}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[params_file],
        ),
    ])
