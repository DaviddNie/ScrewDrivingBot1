import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

# This launch file launches brain, vision, along with a ros bag of the RealSense D435i depth camera
# Use brain_vision_test for testing

def generate_launch_description():

	os.environ["XDG_SESSION_TYPE"] = "xcb"
	
	bag_file_path = '/home/davidnie/4231/ScrewDrivingBot1/camera_bag'

	# Play the bag file using ExecuteProcess
	play_bag = ExecuteProcess(
		cmd=['ros2', 'bag', 'play', bag_file_path, '--clock'],
		output='screen'
	)
	
	brain_node = Node(
			package='brain',
			executable='brain',  # Replace with the actual executable name
			name='brain',
			output='screen',
			parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
		)

	vision = IncludeLaunchDescription(
			PythonLaunchDescriptionSource([os.path.join(
				get_package_share_directory('vision'), 'launch'),
				'/vision_launch.py'])
			)

	return LaunchDescription([
		# Declare launch arguments (optional)
		DeclareLaunchArgument('use_sim_time', default_value='false', 
							   description='Use simulation time if true'),
		play_bag,
		brain_node,
		vision,        
	])