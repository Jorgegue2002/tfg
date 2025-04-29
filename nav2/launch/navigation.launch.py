from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    nav2_params = PathJoinSubstitution([
        FindPackageShare('nav2'), 'config', 'nav2_params.yaml'
    ])
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')

    #Argumentos
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Nombre'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Usa el clock de Gazebo'
    )
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2'), 'maps', 'hospital_floor_1_map.yaml'
        ]),
        description='Path to map .yaml'
    )

    # Lanza slam_toolbox. Se usa cuando no se tiene un mapa.yaml.
    #(Si se va a usar hay que quitar el namespace y el mapa del path, de los argumentos, del nav2_launch y del return.
    #Y descomentar en el return y en el nav2_params.yaml la parte de slam_toolbox )

    # slam_toolbox_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('slam_toolbox'), 'launch', 'online_sync_launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'slam_params_file': nav2_params
    #     }.items()
    # )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': nav2_params
        }.items()
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_map_yaml_cmd,
        #slam_toolbox_launch,
        nav2_launch
    ])
