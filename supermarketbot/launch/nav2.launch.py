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
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    pkg_share_dir = get_package_share_directory('supermarketbot')
    map_yaml_file = os.path.join(pkg_share_dir, 'maps', 'world_map.yaml')
    default_rviz_config_path = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
    nav2_params_file = os.path.join(pkg_share_dir, 'config', 'nav2_params.yaml')
    robot_description_file = os.path.join(pkg_share_dir, 'urdf', 'supermarketbot.xacro')
    
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")
    world_path = os.path.join(pkg_share_dir, 'world', 'world.sdf')
    bridge_config_path = os.path.join(pkg_share_dir, 'config', 'ros_gz_bridge_gazebo.yaml')
    
    default_model_path = ''
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description,
                    {'publish_frequency': 50.0}],
    )
    
    vision_node = Node(
        package=pkg_name,
        executable='vision_node',
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
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
    
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_namespace': "False",
            'slam': 'False',
            'map': f"{map_yaml_file}",
            'use_sim_time': 'True',
            'params_file': f"{nav2_params_file}",
            'autostart': 'True',
            'use_composition': 'True',
            'use_respawn': 'False',
        }.items(),
    )
    
    # Create LaunchDescription with a list of actions
    ld = LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_share_dir),
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation time'),
        DeclareLaunchArgument('model', default_value=default_model_path, description='Robot model'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path, description='RViz config'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        robot_state_publisher_node,
        vision_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        bringup_cmd,
    ])
    
    return ld

if __name__ == '__main__':
    generate_launch_description()