#!/usr/bin/env python3
"""2D bringup: Gazebo Harmonic playground + Map/111 + nav_server + RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav_params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map')
    use_gui = LaunchConfiguration('use_gui')

    default_params = PathJoinSubstitution([
        FindPackageShare('pnc_nav_bringup'), 'config', 'nav_params.yaml'
    ])
    default_map = PathJoinSubstitution([
        FindPackageShare('pnc_nav_bringup'), 'maps', '111', '111.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Navigation parameters file',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz for goal input and path visualization',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml (nav2 OccupancyGrid)',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Start Gazebo GUI',
        ),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.15'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_sim'), 'launch', 'gz_playground.launch.py'
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_gui': use_gui,
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'z_pose': LaunchConfiguration('z_pose'),
            }.items(),
        ),

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
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server']},
                {'use_sim_time': use_sim_time},
                {'bond_timeout': 4.0},
                {'attempt_respawn_reconnection': True},
                {'bond_respawn_max_duration': 10.0},
            ],
        ),

        # Phase 1: fixed map -> odom; AMCL later
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
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
