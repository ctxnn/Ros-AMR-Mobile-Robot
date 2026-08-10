"""
ROS 2 side of the Isaac Sim setup — robot_state_publisher + SLAM Toolbox
+ Nav2, with NO Gazebo/gz_sim/ros_gz_bridge/spawn nodes.

Run this alongside a live Isaac Sim session (Action Graph wired up per
isaac_sim/README.md, simulation pressed Play) instead of
supermarketbot/launch/slam.launch.py or explore.launch.py — Isaac Sim
publishes /clock, /odom, /tf, /scan and subscribes to
/supermarketbot/cmd_vel itself, so nothing here starts a simulator.

Reuses the existing supermarketbot package's URDF and SLAM/Nav2 config
unchanged.

Usage:
    ros2 launch isaac_sim/isaac_nav2_slam.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

pkg_name = 'supermarketbot'


def generate_launch_description():
    pkg_share = get_package_share_directory(pkg_name)
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    slam_pkg_share = get_package_share_directory('slam_toolbox')

    robot_description_file = os.path.join(pkg_share, 'urdf', 'supermarketbot.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    slam_config_path = os.path.join(pkg_share, 'config', 'async_slam_toolbox_cfg.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'slam_cfg.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'publish_frequency': 50.0, 'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg_share, 'launch', 'online_sync_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config_path,
            'use_sim_time': 'true',
        }.items(),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'False',
            'slam': 'True',
            'map': '',
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path),

        robot_state_publisher_node,
        rviz_node,
        slam_launch,
        nav2_bringup,
    ])
