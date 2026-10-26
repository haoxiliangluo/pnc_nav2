#!/usr/bin/env python3
"""起高程图节点：/points → /elevation_map + /traversability_map（需先起 3D 激光仿真）。"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('pnc_nav_map'), 'config', 'elevation.yaml'
    ])
    return LaunchDescription([
        Node(
            package='pnc_nav_map',
            executable='elevation_map_node',
            name='elevation_map_node',
            output='screen',
            parameters=[params],
        ),
    ])
