import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """3D仿真完整导航启动文件 — Go2机器狗 + Velodyne + LIO-SAM"""

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav_params_file = LaunchConfiguration('params_file')
    rviz_arg = LaunchConfiguration('rviz')

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
            'rviz', default_value='true',
            description='Launch RViz'
        ),

        # --- Gazebo仿真 (Go2 + Velodyne激光雷达) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('go2_config'),
                    'launch',
                    'gazebo_velodyne.launch.py'
                ])
            ]),
            launch_arguments={'rviz': 'false'}.items()
        ),

        # --- LIO-SAM (激光惯性里程计 + 3D SLAM) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lio_sam'),
                    'launch',
                    'lidar.launch.py'
                ])
            ]),
            launch_arguments={'rviz': 'false'}.items()
        ),

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
                ('cmd_vel', '/cmd_vel'),
            ]
        ),

        # --- RViz (可选) ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(rviz_arg),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'rviz', 'nav_view.rviz'
            ])],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
