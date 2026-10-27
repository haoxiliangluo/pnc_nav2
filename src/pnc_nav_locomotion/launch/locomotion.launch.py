#!/usr/bin/env python3
"""运动层节点（cmd_vel → /joint_commands）。"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('pnc_nav_locomotion'), 'config', 'locomotion.yaml'
    ])
    return LaunchDescription([
        Node(
            package='pnc_nav_locomotion',
            executable='locomotion_node',
            name='locomotion',
            output='screen',
            parameters=[params],
        ),
    ])
