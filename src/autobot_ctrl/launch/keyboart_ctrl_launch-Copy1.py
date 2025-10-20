from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
	Node(
		package='autobot_ctrl',
		executable='autobot_keyboard',
	),	
	])
