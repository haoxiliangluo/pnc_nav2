"""Compatibility entry: forwards to Gazebo Sim 2D bringup (playground + Map/111)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'maps', '111', '111.yaml'
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'),
                'launch',
                'gz_sim_2d_bringup.launch.py',
            ])),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_gui': LaunchConfiguration('use_gui'),
                'map': LaunchConfiguration('map'),
            }.items(),
        ),
    ])
