import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from pathlib import Path
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import subprocess

def generate_launch_description():
    """Unitree Go2 仿真 — 3D导航验证"""

    pkg_share = FindPackageShare('pnc_nav_simulation')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='multi_floor')

    urdf_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'unitree_go2', 'go2_nav.urdf.xacro'
    ])

    # resolve xacro at launch file generation time
    urdf_path = str(Path(get_package_share_directory('pnc_nav_simulation')) / 'urdf' / 'unitree_go2' / 'go2_nav.urdf.xacro')
    robot_description = subprocess.check_output(['xacro', urdf_path]).decode()

    # write robot_description into a YAML file to avoid YAML parsing issues
    yaml_path = Path(get_package_share_directory('pnc_nav_simulation')) / 'robot_description_go2.yaml'
    yaml_content = 'robot_description: |\n' + '\n'.join(['  ' + line for line in robot_description.splitlines()]) + '\n'
    yaml_path.write_text(yaml_content)

    # also write a plain URDF file to spawn directly
    urdf_out = Path(get_package_share_directory('pnc_nav_simulation')) / 'go2_nav.urdf'
    urdf_out.write_text(robot_description)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='multi_floor',
            description='World: multi_floor, outdoor_terrain, stairs'),

        # --- Gazebo ---
        ExecuteProcess(
            cmd=['gazebo', '--verbose',
                 PathJoinSubstitution([pkg_share, 'worlds', world, '.world']),
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # --- Spawn Go2 ---
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_go2',
            arguments=[
                '-file', str(urdf_out),
                '-entity', 'unitree_go2',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.5',
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
