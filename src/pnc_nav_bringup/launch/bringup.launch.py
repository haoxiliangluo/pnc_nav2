#!/usr/bin/env python3
"""PNC Nav2 统一启动入口（profile 化）。

用法：
  ros2 launch pnc_nav_bringup bringup.launch.py profile:=sim2d
profile:
  sim2d            2D 差速车 + maps/111（Phase 1/2 全链路）
  sim3d            3D 激光车（点云障碍层）
  sim3d_elevation  3D 激光车 + 高程图地形代价（Phase 3）
  real             真机：只起 map_server + nav_server，TF/odom/scan 由外部提供
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PROFILES = {
    'sim2d':            {'kind': 'sim2d', 'scan': 'nav_scan', 'cloud': '',       'elevation': 'false'},
    'sim3d':            {'kind': 'sim3d', 'scan': '',         'cloud': 'points', 'elevation': 'false'},
    'sim3d_elevation':  {'kind': 'sim3d', 'scan': '',         'cloud': 'points', 'elevation': 'true'},
    'sim_go2':          {'kind': 'sim_go2', 'scan': '',       'cloud': '',
                         'elevation': 'false'},
    'sim_go2w':         {'kind': 'sim_go2w', 'scan': '',      'cloud': '',
                         'elevation': 'false'},
    'sim2d_amcl':       {'kind': 'sim2d_amcl', 'scan': 'nav_scan', 'cloud': '',   'elevation': 'false'},
    'sim2d_cartographer': {'kind': 'sim2d_cartographer', 'scan': 'nav_scan', 'cloud': '',
                           'elevation': 'false'},
    'real':             {'kind': 'real',  'scan': 'scan',     'cloud': '',       'elevation': 'false'},
}


def _launch_setup(context, *args, **kwargs):
    profile_name = LaunchConfiguration('profile').perform(context)
    if profile_name not in PROFILES:
        raise RuntimeError(
            f"unknown profile '{profile_name}', available: {list(PROFILES)}")
    profile = PROFILES[profile_name]

    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    actions = [LogInfo(msg=f"[pnc_nav] bringup profile = {profile_name}")]

    localization = LaunchConfiguration('localization').perform(context)

    if profile['kind'] in ('sim_go2', 'sim_go2w'):
        is_wheeled = profile['kind'] == 'sim_go2w'
        robot_ns = 'go2w' if is_wheeled else 'go2'
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_sim'), 'launch',
                'gz_go2w.launch.py' if is_wheeled else 'gz_go2.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_gui': use_gui,
            }.items(),
        ))
        actions.append(Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml}, {'use_sim_time': use_sim_time}],
        ))
        actions.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map', output='screen',
            parameters=[
                {'autostart': True}, {'node_names': ['map_server']},
                {'use_sim_time': use_sim_time}, {'bond_timeout': 4.0},
                {'attempt_respawn_reconnection': True},
                {'bond_respawn_max_duration': 10.0},
            ],
        ))
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ))
        actions.append(Node(
            package='pnc_nav_core', executable='nav_server_node', name='nav_server',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
                # Go2 模型的 base 链接叫 "base"
                {'robot_frame': 'base'},
                {'local_costmap.scan_topic': ''},
                {'local_costmap.cloud_topic': ''},
                # 轮足平台：速度/转向能力有限，跟踪器放慢；命中"轮足"才覆盖
                {'path_tracker.max_linear_vel': 0.25 if is_wheeled else 0.30},
                {'path_tracker.max_angular_vel': 0.8 if is_wheeled else 1.0},
                {'path_tracker.lookahead_distance': 0.6 if is_wheeled else 0.8},
                {'path_tracker.min_linear_vel': 0.10 if is_wheeled else 0.03},
                {'goal_tolerance_dist': 0.25 if is_wheeled else 0.15},
                # 轮足用 DWA（转向更平滑，不像 PP 那样追前视点）
                {'local_planner_plugin':
                    'pnc_nav_planners::DWA2D' if is_wheeled else ''},
                {'path_tracker_plugin':
                    '' if is_wheeled else 'pnc_nav_planners::PurePursuit2D'},
                {'local_planner.max_vel_x': 0.25},
                {'local_planner.max_vel_theta': 0.8},
                {'progress_timeout': 12.0 if is_wheeled else 8.0},
            ],
        ))
        actions.append(Node(
            package='pnc_nav_locomotion', executable='locomotion_node',
            name='locomotion', output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('pnc_nav_locomotion'), 'config',
                    'locomotion_go2w.yaml' if is_wheeled else 'locomotion_go2.yaml']),
                {'use_sim_time': use_sim_time},
            ],
        ))
        actions.append(Node(
            package='pnc_nav_locomotion', executable='joint_command_bridge_node',
            name='joint_command_bridge', output='screen',
            parameters=[{'joint_bridge.prefix': '/model/' + robot_ns + '/joint'},
                        {'use_sim_time': use_sim_time}],
        ))
    elif profile['kind'] == 'sim2d_cartographer':
        # 建图模式：Cartographer 提供 map→odom 与 /map（不启动 nav_server）
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'launch', 'cartographer_slam.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_gui': use_gui,
                'use_rviz': use_rviz,
            }.items(),
        ))
    elif profile['kind'] == 'sim2d_amcl':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'launch', 'amcl_localization.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_gui': use_gui,
                'use_rviz': use_rviz,
                'params_file': params_file,
                'map': map_yaml,
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
            }.items(),
        ))
    elif profile['kind'] == 'sim2d':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'launch', 'gz_sim_2d_bringup.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_gui': use_gui,
                'use_rviz': use_rviz,
                'params_file': params_file,
                'map': map_yaml,
            }.items(),
        ))
    elif profile['kind'] == 'sim3d':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'launch',
                'gz_sim_3d_lidar_bringup.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_gui': use_gui,
                'use_rviz': use_rviz,
                'params_file': params_file,
                'map': map_yaml,
                'use_elevation': profile['elevation'],
            }.items(),
        ))
    else:  # real：地图 + 导航；TF/odom/scan 由外部提供（static=外部给 map->odom，amcl=本节点给）
        const_arg = {'yaml_filename': map_yaml}
        actions.append(Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[const_arg, {'use_sim_time': use_sim_time}],
        ))
        managed = ['map_server']
        if localization == 'amcl':
            actions.append(Node(
                package='nav2_amcl', executable='amcl', name='amcl', output='screen',
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare('pnc_nav_bringup'), 'config', 'amcl_params.yaml']),
                    {'use_sim_time': use_sim_time},
                    {'set_initial_pose': True},
                    {'initial_pose.x': LaunchConfiguration('initial_pose_x')},
                    {'initial_pose.y': LaunchConfiguration('initial_pose_y')},
                    {'initial_pose.yaw': LaunchConfiguration('initial_pose_yaw')},
                ],
            ))
            managed.append('amcl')
        actions.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map', output='screen',
            parameters=[
                {'autostart': True}, {'node_names': managed},
                {'use_sim_time': use_sim_time}, {'bond_timeout': 4.0},
                {'attempt_respawn_reconnection': True},
                {'bond_respawn_max_duration': 10.0},
            ],
        ))
        actions.append(Node(
            package='pnc_nav_core', executable='nav_server_node', name='nav_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ))

    # 健康监控：每个 profile 都在跑（scan/cloud 话题按 profile 区分）
    actions.append(Node(
        package='pnc_nav_diagnostics', executable='health_monitor_node',
        name='health_monitor', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'health.period': 2.0},
            {'health.scan_topic': profile['scan']},
            {'health.cloud_topic': profile['cloud']},
            {'health.check_action_server': profile['kind'] != 'sim2d_cartographer'},
            {'health.require_map': profile['kind'] != 'sim2d_cartographer'},
        ],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='sim2d',
                              description='sim2d | sim2d_amcl | sim2d_cartographer | sim3d | '
                                          'sim3d_elevation | sim_go2 | sim_go2w | real'),
        DeclareLaunchArgument('localization', default_value='static',
                              description='static（固定 map->odom）| amcl（AMCL 提供 map->odom）'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'config', 'nav_params.yaml'])),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'maps', '111', '111.yaml'])),
        OpaqueFunction(function=_launch_setup),
    ])
