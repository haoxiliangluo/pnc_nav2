#!/usr/bin/env python3
"""Gazebo Harmonic playground + simple stock DiffDrive robot + ros_gz bridge."""

import os
import tempfile
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def _materialize_robot_urdf(xacro_path: str) -> tuple[str, str]:
    urdf_file = os.path.join(tempfile.gettempdir(), 'pnc_nav_sim_diff_drive.urdf')
    subprocess.run(['xacro', xacro_path, '-o', urdf_file], check=True)
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()
    return urdf_file, robot_description


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('pnc_nav_sim')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # 可选 urdf / bridge：默认 2D（diff_drive + bridge.yaml），3D 传 diff_drive_3d + bridge_3d
    urdf_name = LaunchConfiguration('robot_urdf').perform(context)
    bridge_name = LaunchConfiguration('bridge_config').perform(context)
    xacro_path = os.path.join(pkg_share, 'urdf', urdf_name)
    bridge_config = os.path.join(pkg_share, 'params', bridge_name)
    world_path = os.path.join(pkg_share, 'worlds', 'playground.sdf')
    urdf_file, robot_description = _materialize_robot_urdf(xacro_path)

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    use_gui = LaunchConfiguration('use_gui')

    gz_args_str = f'-r -v 2 -s {world_path}'

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': gz_args_str,
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'pnc_diff_drive',
                '-file', urdf_file,
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
                '-allow_renaming', 'true',
            ],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_description},
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
            output='screen',
        ),
    ]

    if use_gui.perform(context) == 'true':
        actions.append(
            ExecuteProcess(
                cmd=['gz', 'sim', '-g'],
                name='gazebo_gui',
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('pnc_nav_sim')
    gz_paths = [
        os.path.dirname(pkg_share),
        pkg_share,
        os.path.join(pkg_share, 'worlds'),
        os.path.join(pkg_share, 'models'),
    ]
    set_env = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join(gz_paths),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.08'),
        DeclareLaunchArgument('robot_urdf', default_value='diff_drive.urdf.xacro'),
        DeclareLaunchArgument('bridge_config', default_value='bridge.yaml'),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Attach Gazebo GUI after server starts',
        ),
        set_env,
        OpaqueFunction(function=_launch_setup),
    ])
