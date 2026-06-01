import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """差速小车仿真 — Phase 1 2D导航验证"""

    pkg_share = FindPackageShare('pnc_nav_simulation')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='simple_maze')

    # URDF
    urdf_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'diff_drive', 'diff_drive.urdf.xacro'
    ])

    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='simple_maze',
            description='World file name (without .world extension)'),

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

        # --- Spawn Robot ---
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_diff_drive',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'diff_drive_robot',
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
