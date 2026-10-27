#!/usr/bin/env python3
"""四足链路 demo：四足模型 + 运动层（FakeTrot）+ 关节指令桥。

跑起来后：
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
  ros2 topic echo /joint_states
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pnc_nav_sim'), 'launch', 'gz_quad.launch.py'])),
            launch_arguments={'use_gui': use_gui}.items(),
        ),
        Node(
            package='pnc_nav_locomotion',
            executable='locomotion_node',
            name='locomotion',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('pnc_nav_locomotion'), 'config', 'locomotion.yaml'])],
        ),
        Node(
            package='pnc_nav_locomotion',
            executable='joint_command_bridge_node',
            name='joint_command_bridge',
            output='screen',
            parameters=[{'joint_bridge.prefix': '/model/quad/joint'}],
        ),
    ])
