from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """2D仿真验证启动文件 — 用于验证整体框架和接口"""

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav_params_file = LaunchConfiguration('params_file')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')

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
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz for goal input and path visualization'
        ),

        # --- Gazebo 仿真 (2D差速小车) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_sim'), 'launch', 'diff_drive_sim.launch.py'
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world': world,
            }.items()
        ),

        # Gazebo diff-drive publishes odom -> base_footprint. Phase 1 uses
        # a fixed map -> odom transform so planner paths live in the map frame.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # --- 导航服务器 ---
        Node(
            package='pnc_nav_core',
            executable='nav_server_node',
            name='nav_server',
            output='screen',
            parameters=[
                nav_params_file,
                {'use_sim_time': use_sim_time}
            ]
        ),

        # --- RViz 可视化 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
