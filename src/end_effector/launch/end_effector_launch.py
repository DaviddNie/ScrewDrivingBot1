from launch import LaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([
        # Arduino serial communication node
        Node(
            package='end_effector',  
            executable='arduino_serial', 
            name='arduino_serial', 
            output='screen', 
        ),
        
        # End Effector node
        Node(
            package='end_effector', 
            executable='end_effector',  
            name='end_effector', 
            output='screen',  
        ),
    ])
