# gazebo_spawn_sdf.launch
from inspect import Arguments
import os
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # sdf_file_path = "/home/luqman/ros2_workspace/src/custom_models/models/traffic_bar_stand/model.sdf"
    package_share_path=get_package_share_directory("custom_models")
    sdf_file_path = os.path.join(package_share_path,'models','traffic_bar_stand','model.sdf')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

  Node(
            package='custom_models',
            executable='sdf_spawner_node',
            name='sdf_spawner',
            output='screen',
            arguments=[sdf_file_path,"Traffic_Stand",str(0),str(0),str(0)]),
    
    Node(
            package='custom_models',
            executable='light.bash',
            name='Light_Spawner',
            output='screen'),
    
    ])