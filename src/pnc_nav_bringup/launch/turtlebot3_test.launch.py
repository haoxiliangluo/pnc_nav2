from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """TurtleBot3测试启动文件 - 验证AStar2D和PurePursuit3D"""

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # TurtleBot3模型选择
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    # 参数文件
    nav_params_file = PathJoinSubstitution([
        FindPackageShare('pnc_nav_bringup'), 'config', 'nav_params.yaml'
    ])

    # TurtleBot3地图文件
    map_file = '/home/hao/pnc_nav2/third_party/turtlebot3/turtlebot3_navigation2/map/map.yaml'

    # TurtleBot3 Gazebo world
    turtlebot3_gazebo_launch = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        # 启动TurtleBot3 Gazebo仿真
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_gazebo_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # 地图服务器
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_file},
                {'use_sim_time': use_sim_time}
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server']},
                {'use_sim_time': use_sim_time},
                {'bond_timeout': 4.0},
                {'attempt_respawn_reconnection': True},
                {'bond_respawn_max_duration': 10.0}
            ]
        ),

        # map -> odom 静态TF (Phase 1简化版，后续用AMCL替代)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # PNC Nav Server
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

        # RViz2可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'rviz', 'nav_view.rviz'
            ])],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('use_rviz', default='true'))
        ),
    ])
