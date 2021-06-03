from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim_1',
            executable='turtlesim_node',
            name='sim_1'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim_2',
            executable='turtlesim_node',
            name='sim_2'
        ),

    ])