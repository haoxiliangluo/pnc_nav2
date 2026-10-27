#!/usr/bin/env python3
"""极简四足 demo 仿真：workspace world + quadruped.urdf.xacro + bridge_quad.yaml。

只用于验证 `cmd_vel → 运动层 → 关节` 链路；不含导航栈。
"""

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
                'robot_urdf': 'quadruped.urdf.xacro',
                'bridge_config': 'bridge_quad.yaml',
                'robot_name': 'quad',
                'z_pose': '0.45',
            }.items(),
        ),
    ])
