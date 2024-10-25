from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Arduino serial communication node
        Node(
            package='end-effector',  
            executable='arduino_serial.py', 
            name='arduino_serial_node', 
            output='screen', 
        ),
        
        # End Effector node
        Node(
            package='end-effector', 
            executable='end_effector.py',  
            name='end_effector_node', 
            output='screen',  
        ),
    ])
