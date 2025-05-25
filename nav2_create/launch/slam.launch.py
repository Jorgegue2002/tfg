from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_create_dir = get_package_share_directory('nav2_create')  #Este paquete

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': os.path.join(nav2_create_dir, 'config', 'mapper_params_online_async.yaml'),
                'use_sim_time': 'true',
            }.items()
        )
    ])
