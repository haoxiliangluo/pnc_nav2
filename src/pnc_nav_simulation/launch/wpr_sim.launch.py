import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """WPR (启智) 机器人仿真 — 轮式2D/2.5D导航验证"""

    pkg_share = FindPackageShare('pnc_nav_simulation')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='office')

    urdf_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'wpr', 'wpr_nav.urdf.xacro'
    ])

    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='office',
            description='World: office, simple_maze'),

        # --- Gazebo ---
        ExecuteProcess(
            cmd=['gazebo', '--verbose',
                 PathJoinSubstitution([pkg_share, 'worlds', world, '.world']),
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # --- Robot State Publisher ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }]
        ),

        # --- Spawn WPR ---
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_wpr',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'wpr_robot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
            ],
            output='screen'
        ),

        # --- Joint State Publisher ---
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
