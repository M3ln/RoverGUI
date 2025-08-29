import os
from launch import LaunchDescription
from launch_ros.actions import Node
#from launch_ros.parameter_descriptions import Parameter
from launch import LaunchDescription

def generate_launch_description():
	joystick_initializer = Node(package="joy",
				    executable="joy_node",
				    remappings=[('/joy', '/robot/joy')],
				    parameters=[{'autorepeat_rate' : 0.0}])
	robot_gui_activator = Node(package="robot_gui",
				   executable = "robot_gui_node",
				   output="screen")

	return LaunchDescription([
		joystick_initializer,
		robot_gui_activator
	])
