import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """3D仿真完整导航启动文件 — 机器狗 + 3D环境"""

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav_params_file = LaunchConfiguration('params_file')

    default_params = PathJoinSubstitution([
        FindPackageShare('pnc_nav_bringup'), 'config', 'nav_params.yaml'
    ])

    return LaunchDescription([
        # --- 启动参数 ---
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Navigation parameters file'
        ),
        DeclareLaunchArgument(
            'world', default_value='multi_floor',
            description='Gazebo world (multi_floor, outdoor_terrain, stairs)'
        ),
        DeclareLaunchArgument(
            'robot_model', default_value='go2',
            description='Robot model (go2, a1, custom)'
        ),

        # --- Gazebo 仿真 (3D世界 + 机器狗) ---
        # TODO: IncludeLaunchDescription for gazebo world + quadruped spawn

        # --- LiDAR 里程计 (Fast-LIO2) ---
        # TODO: Node for lidar odometry

        # --- 地图服务 (OctoMap) ---
        # TODO: Node for octomap_server

        # --- 可通行性分析 ---
        # TODO: Node for traversability analysis

        # --- 导航服务器 (3D模式) ---
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
                ('cmd_vel', '/locomotion/cmd_vel'),
            ]
        ),

        # --- 步态控制器 ---
        # TODO: Node for locomotion controller (RL policy inference)

        # --- RViz ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'rviz', 'nav_3d.rviz'
            ])],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
