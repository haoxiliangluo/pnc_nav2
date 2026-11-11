#!/usr/bin/env python3
"""起 octomap_server：点云 → 3D 占据栅格（/octomap_binary、/octomap_full、/projected_map）。

默认输入 `/lio_sam/mapping/cloud_registered`（LIO-SAM 配准后的世界系点云）。
没有 LIO-SAM 时可用 cloud_topic:=points（但需要 map->velodyne 的 TF 链路）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('pnc_nav_map'), 'config', 'octomap.yaml'
    ])
    cloud_topic = LaunchConfiguration('cloud_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_map_to_odom = LaunchConfiguration('publish_map_to_odom')

    return LaunchDescription([
        DeclareLaunchArgument('cloud_topic', default_value='/lio_sam/mapping/cloud_registered'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_map_to_odom', default_value='true',
                              description='LIO-SAM 的点云在 odom 帧，补一个恒等 map->odom 让 octomap 能转过去'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom_octomap',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(publish_map_to_odom),
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[('/cloud_in', cloud_topic)],
        ),
    ])
