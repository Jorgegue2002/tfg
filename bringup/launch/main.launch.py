import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, ExecuteProcess, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Default map poses (if needed for initial robot pose)
MAP_POSES = {
    'hospital': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'R': 0.0, 'P': 0.0, 'Y': 0.0},
}


def generate_launch_description():
    # Paths to packages
    pkg_hospital = get_package_share_directory('hospital')
    pkg_create3_sim = get_package_share_directory('create3_sim')
    pkg_nav2 = get_package_share_directory('nav2_create')

    # Launch configurations
    use_slam = LaunchConfiguration('use_slam')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam_params = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    world_file = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Declare arguments
    declare_use_slam = DeclareLaunchArgument('use_slam', default_value='false', description='Use SLAM toolbox')
    declare_map = DeclareLaunchArgument('map', default_value=os.path.join(pkg_hospital, 'maps', 'hospital.yaml'), description='Full path to map yaml')
    declare_params = DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_nav2, 'config', 'nav2_params.yaml'), description='Path to Nav2 parameters')
    declare_slam_params = DeclareLaunchArgument('slam_params_file', default_value=os.path.join(pkg_nav2, 'config', 'mapper_params_online_async.yaml'), description='Path to SLAM toolbox parameters')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz')
    declare_rviz_config = DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg_nav2, 'config', 'nav2_rviz.yaml'), description='RViz config file')
    declare_world = DeclareLaunchArgument('world', default_value=os.path.join(pkg_hospital, 'worlds', 'hospital.sdf'), description='Gazebo world file')
    declare_robot_name = DeclareLaunchArgument('robot_name', default_value='create3', description='Robot name')
    declare_robot_sdf = DeclareLaunchArgument('robot_sdf', default_value=os.path.join(pkg_create3_sim, 'urdf', 'create3.urdf.xacro'), description='Create3 xacro file')
    declare_initial_pose = [
        DeclareLaunchArgument('x_pose', default_value=str(MAP_POSES['hospital']['x']), description='initial x'),
        DeclareLaunchArgument('y_pose', default_value=str(MAP_POSES['hospital']['y']), description='initial y'),
        DeclareLaunchArgument('z_pose', default_value=str(MAP_POSES['hospital']['z']), description='initial z'),
        DeclareLaunchArgument('roll', default_value=str(MAP_POSES['hospital']['R']), description='roll'),
        DeclareLaunchArgument('pitch', default_value=str(MAP_POSES['hospital']['P']), description='pitch'),
        DeclareLaunchArgument('yaw', default_value=str(MAP_POSES['hospital']['Y']), description='yaw'),
    ]

    # Robot state publisher
    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': Command(['xacro ', robot_sdf])}
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Spawn robot in Gazebo
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_create3_sim, 'launch', 'spawn_create3.launch.py')),
        launch_arguments={
            'robot_name': robot_name,
            'robot_sdf': robot_sdf,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }.items()
    )

    # Gazebo server and client
    world_sdf = tempfile.mktemp(prefix='create3_', suffix='.sdf')
    world_xacro = ExecuteProcess(cmd=['xacro', '-o', world_sdf, world_file])
    gazebo_server = ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', world_sdf], output='screen', condition=IfCondition(use_sim_time))
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        condition=IfCondition(PythonExpression([use_sim_time, ' and not false'])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items()
    )
    cleanup = RegisterEventHandler(event_handler=OnShutdown(on_shutdown=[OpaqueFunction(function=lambda context: os.remove(world_sdf))]))

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'rviz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'rviz_config': rviz_config, 'use_sim_time': use_sim_time}.items()
    )

    # SLAM or Nav2 bringup
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'slam.launch.py')),
        condition=IfCondition(use_slam),
        launch_arguments={'slam_params_file': slam_params, 'use_sim_time': use_sim_time}.items()
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'nav2.launch.py')),
        condition=IfCondition(PythonExpression(['not ', use_slam])),
        launch_arguments={'params_file': params_file, 'map': map_yaml, 'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()
    # Add all declarations
    ld.add_action(declare_use_slam)
    ld.add_action(declare_map)
    ld.add_action(declare_params)
    ld.add_action(declare_slam_params)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_world)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_robot_sdf)
    for decl in declare_initial_pose:
        ld.add_action(decl)

    # Add actions
    ld.add_action(world_xacro)
    ld.add_action(cleanup)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_robot)
    ld.add_action(rsp_node)
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)
    ld.add_action(rviz)

    return ld
