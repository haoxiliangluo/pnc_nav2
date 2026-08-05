#!/usr/bin/env python3
"""AMCL localization + nav_server on SC40 world (replaces static map->odom TF).

Usage:
  ros2 launch pnc_nav_bringup amcl_localization.launch.py map:=<yaml>
  # 默认地图: maps/sc40/sc40.yaml (Cartographer 建图产物)
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
    nav_params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    default_params = PathJoinSubstitution([
        FindPackageShare('pnc_nav_bringup'), 'config', 'nav_params.yaml'
    ])
    default_map = PathJoinSubstitution([
        FindPackageShare('pnc_nav_bringup'), 'maps', 'sc40', 'sc40.yaml'
    ])
    default_amcl_params = PathJoinSubstitution([
        FindPackageShare('pnc_nav_bringup'), 'config', 'amcl_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.08'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('map', default_value=default_map),

        # SC40 world + pnc_diff_drive + ros_gz bridge
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

        # Static map + lifecycle (Nav2 standard)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_yaml},
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']},
                {'use_sim_time': use_sim_time},
                {'bond_timeout': 4.0},
            ],
        ),

        # AMCL: /nav_scan + odom -> map->odom TF (replaces static TF)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[default_amcl_params],
        ),

        Node(
            package='pnc_nav_core',
            executable='nav_server_node',
            name='nav_server',
            output='screen',
            parameters=[
                nav_params_file,
                {'use_sim_time': use_sim_time},
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
