#!/usr/bin/env python3
"""轮足（Go2-W）链路 demo：模型 + 运动层（腿站立/轮差速）+ 关节桥。"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_sim'), 'launch', 'gz_go2w.launch.py'])),
            launch_arguments={'use_gui': use_gui}.items(),
        ),
        Node(
            package='pnc_nav_locomotion',
            executable='locomotion_node',
            name='locomotion',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('pnc_nav_locomotion'), 'config', 'locomotion_go2w.yaml'])],
        ),
        Node(
            package='pnc_nav_locomotion',
            executable='joint_command_bridge_node',
            name='joint_command_bridge',
            output='screen',
            parameters=[{'joint_bridge.prefix': '/model/go2w/joint'}],
        ),
    ])
