import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Unitree Go2 — classic Gazebo 3D navigation validation.

    Loads a pnc_nav_sim world in classic Gazebo and spawns the go2_classic
    model (real Go2 trunk mesh + planar_move locomotion abstraction + Mid360/IMU).
    This stands up the simulated environment + sensors only; bring up the
    navigation stack separately (e.g. pnc_nav_bringup/sim_3d_bringup.launch.py).
    """

    pkg_share = FindPackageShare('pnc_nav_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    urdf_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'unitree_go2', 'go2_classic.urdf.xacro'
    ])
    world_file = PathJoinSubstitution([pkg_share, 'worlds', [world, '.world']])

    # robot_description is resolved lazily by xacro at launch time (no temp files)
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world', default_value='multi_floor',
            description='World file (without .world): multi_floor, simple_maze'),

        # --- Classic Gazebo ---
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # --- Robot State Publisher (also serves /robot_description for spawn) ---
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

        # --- Spawn Go2 from the published robot_description topic ---
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_go2',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'unitree_go2',
                '-x', '0.0', '-y', '0.0', '-z', '0.5',
            ],
            output='screen'
        ),

        # --- Joint State Publisher (static feet, but keeps TF tree complete) ---
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
