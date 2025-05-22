from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Definir la ruta al archivo del mundo
    world_path = PathJoinSubstitution([
        get_package_share_directory('hospital'),
        'worlds',
        'hospital.sdf'
    ])

    # Incluir el launch oficial de ros_gz_sim para lanzar el mundo
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='Path to the world file'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('ros_gz_sim'), '/launch/gz_sim.launch.py']
            ),
            launch_arguments={
                'gz_args': [LaunchConfiguration('world')],
            }.items(),
        ),
    ])
