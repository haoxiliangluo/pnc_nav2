import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """2D仿真验证启动文件 — 用于验证整体框架和接口"""

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav_params_file = LaunchConfiguration('params_file')

    default_params = PathJoinSubstitution([
        FindPackageShare('pnc_nav_bringup'), 'config', 'nav_params.yaml'
    ])

    return LaunchDescription([
        # --- 启动参数声明 ---
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Navigation parameters file'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='simple_maze',
            description='Gazebo world name'
        ),

        # --- Gazebo 仿真 (2D差速小车) ---
        # TODO: IncludeLaunchDescription for gazebo + robot spawn

        # --- 导航服务器 ---
        Node(
            package='pnc_nav_core',
            executable='nav_server_node',
            name='nav_server',
            output='screen',
            parameters=[
                nav_params_file,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('cmd_vel', '/diff_drive/cmd_vel'),
                ('odom', '/diff_drive/odom'),
            ]
        ),

        # --- 地图服务器 (2D模式使用Nav2 map_server) ---
        # TODO: Node for map_server

        # --- RViz 可视化 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'rviz', 'nav_2d.rviz'
            ])],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
