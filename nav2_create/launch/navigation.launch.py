from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node


def generate_launch_description():
    # Directorios
    bringup_dir = get_package_share_directory('nav2_bringup')
    nav_dir = get_package_share_directory('nav2_create')

    launch_dir = os.path.join(bringup_dir, 'launch')

    # Argumentos
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )
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
        default_value=os.path.join(nav_dir, 'config', 'nav2_params.yaml'),
        description='File with Nav2 parameters'
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(nav_dir, 'config', 'mapper_params_online_async.yaml'),
        description='File with SLAM Toolbox parameters'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_dir, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file'
    )
    use_localization_arg = DeclareLaunchArgument(
        'use_localization', default_value='True', description='Enable localization'
    )

    # Lanzar slam_toolbox
    bringup_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        condition=IfCondition(LaunchConfiguration('slam')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'slam': 'True',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('slam_params_file'),
        }.items()
    )

    # Lanzar navigation
    bringup_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('slam')])),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'slam': 'False',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'map': LaunchConfiguration('map'),
            'use_localization': LaunchConfiguration('use_localization'),
        }.items()
    )

    # Publica pose inicial
    initial_pose_pub = Node(
        package='mis_nodos',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen'
    )

    # mux_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(nav_dir, 'launch', 'twist_mux.launch.py'))
    # )

    ld = LaunchDescription([
        namespace_arg,
        slam_arg,
        use_sim_time_arg,
        autostart_arg,
        nav2_params_arg,
        slam_params_arg,
        map_arg,
        use_localization_arg,
        bringup_slam,
        bringup_nav,
        initial_pose_pub,
        # mux_launch
    ])

    return ld
