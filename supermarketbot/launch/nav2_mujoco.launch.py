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
    bringup_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    default_rviz_config_path = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')

    map_yaml_file = os.path.join(pkg_share, 'maps', 'mujoco_map.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    robot_description_file = os.path.join(pkg_share, 'urdf', 'supermarketbot.xacro')
    robot_description = {'robot_description': xacro.process_file(robot_description_file).toxml()}

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

    # Nav2 publishes /cmd_vel; mujoco_bridge listens on /supermarketbot/cmd_vel
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        arguments=['/cmd_vel', '/supermarketbot/cmd_vel'],
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'False',
            'slam': 'False',
            'map': map_yaml_file,
            # nothing publishes /clock — mujoco_bridge stamps with wall time
            'use_sim_time': 'False',
            'params_file': nav2_params_file,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path,
                              description='RViz config'),
        robot_state_publisher_node,
        mujoco_bridge_node,
        rviz_node,
        cmd_vel_relay,
        bringup_cmd,
    ])
