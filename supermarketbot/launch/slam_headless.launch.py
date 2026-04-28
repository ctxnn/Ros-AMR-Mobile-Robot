"""
Lightweight SLAM launch — optimised for WSL2 performance.

Same as slam.launch.py but:
  - Runs Gazebo **headless** (no GUI) — huge performance gain on WSL2
  - Still launches RViz so you can see the map building
  - Camera bridge is removed (not needed for mapping)
  - Point cloud bridge removed (LaserScan is sufficient for SLAM)

Usage:
    # Terminal 1 — launch headless SLAM
    ros2 launch supermarketbot slam_headless.launch.py

    # Terminal 2 — auto exploration OR manual teleop
    ros2 run supermarketbot auto_explorer
    # OR
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r /cmd_vel:=/supermarketbot/cmd_vel

    # Terminal 3 — save map when done
    ros2 run nav2_map_server map_saver_cli \\
        -f ~/s_ws/src/supermarketbot/maps/world_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

pkg_name = 'supermarketbot'


def generate_launch_description():

    pkg_share = get_package_share_directory(pkg_name)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    gz_sim_launch_file = os.path.join(
        ros_gz_sim_share, 'launch', 'gz_sim.launch.py'
    )
    default_rviz_config_path = os.path.join(
        pkg_share, 'rviz', 'slam_cfg.rviz'
    )
    world_path = os.path.join(pkg_share, 'world', 'world.sdf')
    bridge_config_path = os.path.join(
        pkg_share, 'config', 'ros_gz_bridge_gazebo.yaml'
    )
    slam_config_path = os.path.join(
        pkg_share, 'config', 'async_slam_toolbox_cfg.yaml'
    )

    robot_description_file = os.path.join(
        pkg_share, 'urdf', 'supermarketbot.xacro'
    )
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {
        'robot_description': robot_description_config.toxml()
    }

    slam_pkg_share = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(
        slam_pkg_share, 'launch', 'online_sync_launch.py'
    )

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

    # Gazebo server — HEADLESS (no GUI, no rendering overhead)
    # The -s flag runs the server only; -r starts unpaused.
    # --headless-rendering disables the GPU render pipeline entirely
    # since we only need physics + LiDAR raycasting for SLAM.
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={
            'gz_args': '-r -s ' + world_path,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Minimal bridge — only topics needed for SLAM (no camera, no pointcloud)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config_path}],
    )

    # Spawn the robot
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

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_share, os.pardir),
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

        # NO Gazebo GUI — headless only
        robot_state_publisher_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        spawn_entity,

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'slam_params_file': slam_config_path,
                'use_sim_time': 'true',
            }.items(),
        ),
    ])
