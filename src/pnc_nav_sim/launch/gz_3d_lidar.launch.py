#!/usr/bin/env python3
"""3D LiDAR (Velodyne-style 16-line) bringup on SC40 world.

Same robot as the 2D one, but with a Velodyne VLP-16 style gpu_lidar
publishing /points (PointCloud2). Use web_teleop to drive it.

Usage:
  ros2 launch pnc_nav_sim gz_3d_lidar.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('use_gui')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.08'),

        # SC40 world + 3D-lidar robot + 3D bridge (points -> PointCloud2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_sim'), 'launch', 'gz_playground.launch.py'
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_gui': use_gui,
                'x_pose': x_pose,
                'y_pose': y_pose,
                'z_pose': z_pose,
                'robot_urdf': 'diff_drive_3d.urdf.xacro',
                'bridge_config': 'bridge_3d.yaml',
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('pnc_nav_sim'), 'rviz', 'view_3d.rviz'
            ])],
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
