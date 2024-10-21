from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='vision',
            executable='vision_server',
            name='vision_server',
            output='screen',
        )
    ])