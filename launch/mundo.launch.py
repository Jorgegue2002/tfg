from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('tfg'),
        'hospital',
        'worlds',
        'hospital.sdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='Path to world file'
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', LaunchConfiguration('world')],
            output='screen'
        )
    ])
