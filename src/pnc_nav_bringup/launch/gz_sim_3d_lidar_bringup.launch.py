#!/usr/bin/env python3
"""3D 激光 bringup: Gazebo + 差速车(Velodyne 16线) + maps/111 + nav_server + RViz。

与 2D 版区别：
- 用 pnc_nav_sim/gz_3d_lidar.launch.py（diff_drive_3d + bridge_3d，输出 /points）
- 打开局部代价地图的点云障碍层（local_costmap.cloud_topic:=points）
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
    nav_params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map')
    use_gui = LaunchConfiguration('use_gui')
    cloud_topic = LaunchConfiguration('cloud_topic')
    use_elevation = LaunchConfiguration('use_elevation')
    world = LaunchConfiguration('world')

    default_params = PathJoinSubstitution([
        FindPackageShare('pnc_nav_bringup'), 'config', 'nav_params.yaml'
    ])
    default_map = PathJoinSubstitution([
        FindPackageShare('pnc_nav_bringup'), 'maps', '111', '111.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('cloud_topic', default_value='points',
                              description='3D 点云话题（填 "" 可关闭点云障碍层）'),
        DeclareLaunchArgument('use_elevation', default_value='false',
                              description='true = 启动高程图节点，并让全局规划叠加坡度/台阶代价'),
        DeclareLaunchArgument('world', default_value='playground.sdf',
                              description='pnc_nav_sim/worlds 下的世界文件（如 slope_test.sdf）'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.15'),

        # 3D 激光仿真（diff_drive_3d + Velodyne，输出 /points）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_sim'), 'launch', 'gz_3d_lidar.launch.py'
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_rviz': 'false',
                'use_gui': use_gui,
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'z_pose': LaunchConfiguration('z_pose'),
                'world': world,
            }.items(),
        ),

        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml}, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map', output='screen',
            parameters=[
                {'autostart': True}, {'node_names': ['map_server']},
                {'use_sim_time': use_sim_time}, {'bond_timeout': 4.0},
                {'attempt_respawn_reconnection': True},
                {'bond_respawn_max_duration': 10.0},
            ],
        ),

        # 阶段 1/2：固定 map->odom（还没跑定位）
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 可选：高程图节点（/points → /traversability_map）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_map'), 'launch', 'elevation_map.launch.py'
            ])),
            condition=IfCondition(use_elevation),
        ),

        Node(
            package='pnc_nav_core', executable='nav_server_node', name='nav_server',
            output='screen',
            parameters=[
                nav_params_file,
                {'use_sim_time': use_sim_time},
                {'local_costmap.cloud_topic': cloud_topic},
                {'use_elevation_costmap': use_elevation},
            ],
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'rviz', 'nav_view.rviz'
            ])],
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
