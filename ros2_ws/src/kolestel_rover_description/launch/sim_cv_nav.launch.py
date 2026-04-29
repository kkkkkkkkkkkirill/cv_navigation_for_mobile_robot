import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('kolestel_rover_description')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py'))
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
            'kp': 0.006,
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
        }],
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(period=10.0, actions=[line_follower]),
        TimerAction(period=11.0, actions=[aruco_node]),
        TimerAction(period=12.0, actions=[cv_navigator]),
    ])
