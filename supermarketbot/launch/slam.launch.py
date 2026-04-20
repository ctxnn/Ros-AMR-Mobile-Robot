import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer
import xacro
from launch.conditions import IfCondition, UnlessCondition

pkg_name = 'supermarketbot'

def generate_launch_description():
    
    pkg_share = get_package_share_directory(pkg_name)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")
    default_model_path = os.path.join(pkg_share, 'urdf', 'supermarketbot.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'slam_cfg.rviz')
    world_path = os.path.join(pkg_share, 'world', 'world.sdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'ros_gz_bridge_gazebo.yaml')
    slam_config_path = os.path.join(pkg_share, 'config', 'async_slam_toolbox_cfg.yaml')
    
    robot_description_file = os.path.join(pkg_share, 'urdf', 'supermarketbot.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    slam_pkg_share = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_pkg_share, 'launch', 'online_sync_launch.py')

    # Start Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description,
                    {'publish_frequency': 50.0}],
    )
    # vision_node = Node(
    #     package=pkg_name,
    #     executable='vision_node',
    #     output='screen'
    # )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    # teleop_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_twist_keyboard',
    #     output='screen',
    #     prefix='gnome-terminal --',  # Opens in new terminal window
    #     remappings=[('/cmd_vel', '/supermarketbot/cmd_vel')]
    # )
    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'empty',
            'topic': '/robot_description',
            'entity_name': 'supermarketbot',
            'x': '1.0',
            'y': '1.0',
            'z': '0.65',
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_share),
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation time'),
        DeclareLaunchArgument('model', default_value=default_model_path, description='Robot model'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path, description='RViz config'),

        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        robot_state_publisher_node,
        # vision_node,
        rviz_node,
        # teleop_node,
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        

        # Include SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'slam_params_file': slam_config_path,
                'use_sim_time': 'true'
            }.items()
        )
    ])