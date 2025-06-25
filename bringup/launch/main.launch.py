import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Configurar el nivel de log
    set_log_level = SetLaunchConfiguration('log_level', 'info')

    # Directorios
    create3_launch_dir = get_package_share_directory('irobot_create_gz_bringup')

    nav2_create_dir = get_package_share_directory('nav2_create')

    # Argumentos de lanzamiento de Gazebo + Hospital + RViz

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz.',
        choices=['true', 'false']
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='hospital',
        description='Ignition World'
    )

    # Posición por defecto del robot
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.3',
        description='x component of the robot pose.'
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='-17.0',
        description='y component of the robot pose.'
    )
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.0',
        description='z component of the robot pose.'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='3.14',
        description='yaw component of the robot pose.'
    )

    #Argumentos de lanzamiento de Nav2

    # namespace_arg = DeclareLaunchArgument(
    #     'namespace', default_value='', description='Top-level namespace'
    # )
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='False', description='Run SLAM toolbox if true'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use sim time'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true', description='Auto start nav2'
    )
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(nav2_create_dir, 'config', 'nav2_params.yaml'),
        description='File with Nav2 parameters'
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(nav2_create_dir, 'config', 'mapper_params_online_async.yaml'),
        description='File with SLAM Toolbox parameters'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_create_dir, 'maps', 'hospital.yaml'),
        description='Full path to map yaml file'
    )
    use_localization_arg = DeclareLaunchArgument(
        'use_localization', default_value='True', description='Enable localization'
    )

    gz_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(create3_launch_dir, 'launch', 'create3_gz.launch.py')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_create_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'slam': LaunchConfiguration('slam'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'map': LaunchConfiguration('map'),
            'use_localization': LaunchConfiguration('use_localization'),
        }.items()
    )

    ld = LaunchDescription([
        set_log_level,
        namespace_arg,
        use_rviz_arg,
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        slam_arg,
        use_sim_time_arg,
        autostart_arg,
        nav2_params_arg,
        slam_params_arg,
        map_arg,
        use_localization_arg,
        
        gz_rviz_launch,
        nav2_launch
    ])

    return ld