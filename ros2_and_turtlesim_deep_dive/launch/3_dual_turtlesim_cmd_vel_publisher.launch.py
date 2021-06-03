from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim_1'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim_2'
        ),
        Node(
            package='ros2_and_turtlesim_deep_dive',
            executable='ts_velocity_publisher',
            name='cmd_vel_publisher',
            remappings=[
                ('/turtle1/cmd_vel', '/turtle2/cmd_vel'),
            ]
            
        ),
        Node(
            package='ros2_and_turtlesim_deep_dive',
            executable='ts_velocity_publisher',
            name='cmd_vel_publisher',
        ),
        

    ])