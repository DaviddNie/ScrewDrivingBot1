import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    package_name = 'end_effector_description'
    xacro_path = 'urdf/end_effector.xacro'
    rviz_path = 'rviz/default.rviz'

    xacro_file = os.path.join(get_package_share_directory(package_name), xacro_path)
    xacro_raw_description = xacro.process_file(xacro_file).toxml()

    rviz_file = os.path.join(get_package_share_directory(package_name), rviz_path)

    return LaunchDescription([
        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': xacro_raw_description}]),
        Node(
            name='joint_state_publisher_gui',
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': xacro_raw_description}]),
        Node(
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
			),
    ])