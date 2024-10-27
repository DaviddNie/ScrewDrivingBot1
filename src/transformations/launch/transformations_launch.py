from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='transformations',
            executable='camera_dy_trans',
            name='camera_dy_trans',
            output='screen',
        )
    ])