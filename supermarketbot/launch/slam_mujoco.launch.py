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
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'slam_cfg.rviz')
    slam_config_path = os.path.join(pkg_share, 'config', 'async_slam_toolbox_cfg.yaml')

    robot_description_file = os.path.join(pkg_share, 'urdf', 'supermarketbot.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    slam_pkg_share = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_pkg_share, 'launch', 'online_sync_launch.py')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'publish_frequency': 50.0, 'use_sim_time': False}],
    )

    mujoco_bridge_node = Node(
        package=pkg_name,
        executable='mujoco_bridge',
        name='mujoco_bridge',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False', description='mujoco_bridge runs in real time, not simulated time'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path, description='RViz config'),

        robot_state_publisher_node,
        mujoco_bridge_node,
        rviz_node,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'slam_params_file': slam_config_path,
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
