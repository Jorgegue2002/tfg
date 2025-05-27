from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    nav_dir = get_package_share_directory('nav2_create')

    declare_mux_params = DeclareLaunchArgument(
        'mux_params_file',
        default_value=os.path.join(nav_dir, 'config', 'twist_mux.yaml'),
        description='File with Twist_mux parameters'
    )

    mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[LaunchConfiguration('mux_params_file')],
        #El cmd_vel este es del tipo Twist y el que mueve al robot es de tipo TwistStamped, asi que esto lo hago en un nodo ahora
        # remappings=[
        #     ('/cmd_vel_out', '/cmd_vel')
        # ]
    )

    converter_node = Node(
        package='mis_nodos',
        executable='mux_out_to_cmd_vel',
        name='mux_out_to_cmd_vel',
        output='screen'
    )

    return LaunchDescription([
        declare_mux_params,
        mux_node,
        converter_node
    ])
