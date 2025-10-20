from launch import LaunchDescription
#from launch.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    autobot_app_save_map_node = Node(
        package='autobot_app_save_map',
        executable="server",
        output="screen"
    )

    return LaunchDescription(
        [
            autobot_app_save_map_node,
        ]
    )
