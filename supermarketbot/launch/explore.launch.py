"""
Autonomous exploration launch file.

Combines Gazebo simulation, SLAM Toolbox, and the full Nav2 stack
(in SLAM mode) so the auto_explorer node can send Nav2 goals to
systematically map the entire supermarket without manual teleop.

Usage:
    ros2 launch supermarketbot explore.launch.py
    # Then in another terminal:
    ros2 run supermarketbot auto_explorer
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

pkg_name = 'supermarketbot'


def generate_launch_description():
    pkg_share_dir = get_package_share_directory(pkg_name)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    bringup_launch_dir = os.path.join(nav2_bringup_share, 'launch')

    gz_sim_launch_file = os.path.join(
        ros_gz_sim_share, 'launch', 'gz_sim.launch.py'
    )
    world_path = os.path.join(pkg_share_dir, 'world', 'world.sdf')
    bridge_config_path = os.path.join(
        pkg_share_dir, 'config', 'ros_gz_bridge_gazebo.yaml'
    )
    nav2_params_file = os.path.join(
        pkg_share_dir, 'config', 'nav2_params.yaml'
    )
    robot_description_file = os.path.join(
        pkg_share_dir, 'urdf', 'supermarketbot.xacro'
    )
    default_rviz_config_path = os.path.join(
        nav2_bringup_share, 'rviz', 'nav2_default_view.rviz'
    )

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {
        'robot_description': robot_description_config.toxml()
    }

    # ── Nodes ──────────────────────────────────────────────────────────

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'publish_frequency': 50.0}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={
            'gz_args': '-r ' + world_path + ' --render-engine-server ogre',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config_path}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_supermarketbot',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'supermarketbot',
            '-x', '1.0',
            '-y', '1.0',
            '-z', '0.65',
        ],
    )

    # Relay /cmd_vel (Nav2 output) -> /supermarketbot/cmd_vel (Gazebo input)
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        arguments=['/cmd_vel', '/supermarketbot/cmd_vel'],
    )

    # Nav2 bringup with slam:=True — uses SLAM Toolbox for live mapping
    # instead of loading a static map from disk.
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'False',
            'slam': 'True',           # ← key difference from nav2.launch.py
            'map': '',
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_share_dir, os.pardir),
        ),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'rvizconfig', default_value=default_rviz_config_path,
            description='RViz config',
        ),

        # Gazebo GUI client (ogre1 for WSL2 compatibility)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-g', '--render-engine-gui', 'ogre'],
            output='screen',
        ),

        robot_state_publisher_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        cmd_vel_relay,
        bringup_cmd,
    ])
