#!/usr/bin/env python3
"""Unitree Go2-W（轮足）模型仿真：开源 go2w_description + gz 关节/轮速控制器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('world', default_value='slope_test.sdf'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_sim'), 'launch', 'gz_playground.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_gui': use_gui,
                'world': world,
                'robot_urdf': 'go2w.urdf',
                'bridge_config': 'bridge_go2w.yaml',
                'robot_name': 'go2w',
                'z_pose': '0.45',
                'mesh_search_path': PathJoinSubstitution([
                    FindPackageShare('pnc_nav_sim'), 'go2w']),
                'append_gazebo_file': PathJoinSubstitution([
                    FindPackageShare('pnc_nav_sim'), 'params', 'go2w_joint_plugins.xml']),
            }.items(),
        ),
    ])
