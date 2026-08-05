#!/usr/bin/env python3
"""Cartographer 2D SLAM (mapping) on SC40 world with pnc_diff_drive.

Usage:
  ros2 launch pnc_nav_bringup cartographer_slam.launch.py
  # 键盘/手柄控制机器人移动建图
  # 保存地图:
  #   ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/hao/pnc_nav2/maps/sc40.pbstream'}"
  #   ros2 run cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename /home/hao/pnc_nav2/maps/sc40.pbstream -map_filestem /home/hao/pnc_nav2/maps/sc40
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

    config_dir = PathJoinSubstitution([
        FindPackageShare('pnc_nav_bringup'), 'config', 'cartographer'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.08'),

        # SC40 world + pnc_diff_drive + ros_gz bridge (nav_scan / odom / tf)
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
            }.items(),
        ),

        # Laser stamp monotonization (gz gpu_lidar may repeat stamps)
        Node(
            package='pnc_nav_sim',
            executable='mono_scan_node.py',
            name='mono_scan_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Cartographer SLAM: laser /nav_scan_mono -> map
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('scan', 'nav_scan_mono')],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'pnc_nav_2d.lua',
            ],
        ),

        # Submap -> /map OccupancyGrid
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'resolution': 0.05},
                {'publish_period_sec': 0.5},
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'rviz', 'nav_view.rviz'
            ])],
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
