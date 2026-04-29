import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

pkg_name = 'supermarketbot'

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    pkg_share_dir = get_package_share_directory('supermarketbot')
    map_yaml_file = os.path.join(pkg_share_dir, 'maps', 'supermarket_map.yaml')
    default_rviz_config_path = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
    nav2_params_file = os.path.join(pkg_share_dir, 'config', 'nav2_params.yaml')
    robot_description_file = os.path.join(pkg_share_dir, 'urdf', 'supermarketbot.xacro')
    
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch_file = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
    world_path = os.path.join(pkg_share_dir, 'world', 'world.sdf')
    bridge_config_path = os.path.join(pkg_share_dir, 'config', 'ros_gz_bridge_gazebo.yaml')
    
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
    
    # Gazebo Sim server (Humble-compatible: use IncludeLaunchDescription with gz_sim.launch.py)
    # Pass --render-engine-server ogre to avoid OGRE2 crash on WSL2
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={
            'gz_args': '-r ' + world_path + ' --render-engine-server ogre',
            'on_exit_shutdown': 'true',
        }.items(),
    )
    
    # ROS-Gazebo bridge (Humble-compatible: use parameter_bridge Node)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config_path}],
    )
    
    # Spawn the robot entity into Gazebo
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
    
    # Relay /cmd_vel (Nav2 output) -> /supermarketbot/cmd_vel (Gazebo diff_drive input)
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
            'use_namespace': "False",
            'slam': 'False',
            'map': f"{map_yaml_file}",
            'use_sim_time': 'True',
            'params_file': f"{nav2_params_file}",
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )
    
    # Create LaunchDescription with a list of actions
    ld = LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share_dir, os.pardir)),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation time'),
        DeclareLaunchArgument('model', default_value='', description='Robot model'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path, description='RViz config'),
        # Launch Gazebo GUI client (separate from the server, use ogre1 to avoid OGRE2 crash on WSL2)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-g', '--render-engine-gui', 'ogre'],
            output='screen',
        ),
        robot_state_publisher_node,
        # vision_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        cmd_vel_relay,
        bringup_cmd,
    ])
    
    return ld

if __name__ == '__main__':
    generate_launch_description()