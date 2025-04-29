from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot')
    xacro_path = os.path.join(pkg_share, 'urdf', 'create3.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('name', default_value='create3'),

        # Publica robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': Command(['xacro ', xacro_path])
            }]
        ),

        # Spawnea el robot  en Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_create3',
            output='screen',
            arguments=[
                '-name', LaunchConfiguration('name'),
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw'),
                '-topic', 'robot_description'
            ]
        )
    ])
