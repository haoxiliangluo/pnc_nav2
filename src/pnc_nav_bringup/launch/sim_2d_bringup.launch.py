from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """2D仿真验证启动文件 — 当前复用 TurtleBot3 Phase 1 链路"""

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav_params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    turtlebot3_model = LaunchConfiguration('turtlebot3_model')

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
            'use_rviz',
            default_value='true',
            description='Start RViz for goal input and path visualization'
        ),
        DeclareLaunchArgument(
            'turtlebot3_model',
            default_value='waffle',
            description='TurtleBot3 model: burger, waffle, or waffle_pi'
        ),

        SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model),

        # --- TurtleBot3 Gazebo + map_server 是当前 Phase 1 可复现入口 ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py'
            ])),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': '/home/hao/pnc_nav2/third_party/turtlebot3/turtlebot3_navigation2/map/map.yaml'},
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
                {'use_sim_time': use_sim_time}
            ]
        ),

        # Phase 1 uses a fixed map -> odom transform; AMCL is a later step.
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
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('pnc_nav_bringup'), 'rviz', 'nav_view.rviz'
            ])],
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
