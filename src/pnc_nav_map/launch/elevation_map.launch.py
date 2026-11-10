#!/usr/bin/env python3
"""起高程图节点：/points → /elevation_map + /traversability_map（需先起 3D 激光仿真）。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('pnc_nav_map'), 'config', 'elevation.yaml'
    ])
    robot_frame = LaunchConfiguration('robot_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('robot_frame', default_value='base_link',
                              description='机器人基座 TF（轮足是 base）'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='pnc_nav_map',
            executable='elevation_map_node',
            name='elevation_map_node',
            output='screen',
            parameters=[params,
                        {'elevation.robot_frame': robot_frame},
                        {'use_sim_time': use_sim_time}],
        ),
    ])
