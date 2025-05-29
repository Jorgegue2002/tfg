from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mis_nodos',
            executable='entregador',
            name='entregador',
            output='screen'
        )
    ])
