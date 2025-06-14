# Copyright 2021 Clearpath Robotics, Inc.
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='hospital',
                          description='Ignition World'),
]

#Definir la posicion en la que spawneara el robot
default_pose = {
    'x': '0.3',
    'y': '-17.0',
    'z': '0.0',
    'yaw': '3.14'
}

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(
        pose_element,
        default_value=default_pose.get(pose_element, '0.0'),
        description=f'{pose_element} component of the robot pose.'
    ))

def generate_launch_description():

    # Directories
    pkg_irobot_create_gz_bringup = get_package_share_directory(
        'irobot_create_gz_bringup')

    # Paths
    ignition_launch = PathJoinSubstitution(
        [pkg_irobot_create_gz_bringup, 'launch', 'sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_irobot_create_gz_bringup, 'launch', 'create3_spawn.launch.py'])
    create3_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_gz_bringup, 'launch', 'create3_gz_nodes.launch.py'])

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_rviz', LaunchConfiguration('use_rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))])

    create3_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_nodes_launch]),
        launch_arguments=[])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    ld.add_action(create3_nodes)
    return ld
