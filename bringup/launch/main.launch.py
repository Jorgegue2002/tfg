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
]

for pose_element, default in zip(['x', 'y', 'z', 'yaw'], ['0', '1', '0', '0']):
    ARGUMENTS.append(DeclareLaunchArgument(
        pose_element,
        default_value=default,
        description=f'{pose_element} component of the robot pose.'
    ))


def generate_launch_description():

    # Directories
    pkg_hospital = get_package_share_directory(
        'hospital')
    pkg_irobot_create_gz_bringup = get_package_share_directory(
        'irobot_create_gz_bringup')

    # Paths
    mundo_launch = PathJoinSubstitution(
        [pkg_hospital, 'launch', 'mundo.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_irobot_create_gz_bringup, 'launch', 'create3_gz.launch.py'])

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([mundo_launch]),
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

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    return ld