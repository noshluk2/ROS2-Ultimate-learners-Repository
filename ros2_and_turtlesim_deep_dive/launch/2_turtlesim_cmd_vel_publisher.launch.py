from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node'
        ),
        Node(
            package='ros2_and_turtlesim_deep_dive',
            executable='ts_velocity_publisher',
            name='cmd_vel_publisher',
            output='screen'
        ),

    ])