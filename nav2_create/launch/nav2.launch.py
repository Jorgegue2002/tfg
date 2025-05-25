from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_create_dir = get_package_share_directory('nav2_create')

    map_file = os.path.join(nav2_create_dir, 'maps', 'my_map.yaml')
    params_file = os.path.join(nav2_create_dir, 'config', 'nav2_params.yaml')

    # 1) Servidor de mapa explícito
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]
    )

    # 2) Nodo AMCL para localización
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    # 3) Lifecycle manager para map_server y amcl
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # 4) Navegación (planner, controller, etc.)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # 5) Nodo de pose inicial
    initial_pose_pub = Node(
        package='nav2_create',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen'
    )

    return LaunchDescription([
        map_server,
        amcl_node,
        lifecycle_localization,
        navigation_launch,
        initial_pose_pub,
    ])
