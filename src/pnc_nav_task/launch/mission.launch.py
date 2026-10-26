#!/usr/bin/env python3
"""任务层节点（需先起 bringup.launch.py 里的 nav_server）。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    auto_start = LaunchConfiguration('auto_start')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('pnc_nav_task'), 'config', 'mission.yaml'])),
        DeclareLaunchArgument('auto_start', default_value='false'),
        Node(
            package='pnc_nav_task',
            executable='mission_executor_node',
            name='mission_executor',
            output='screen',
            parameters=[
                params,
                {'mission.auto_start': auto_start},
            ],
        ),
    ])
