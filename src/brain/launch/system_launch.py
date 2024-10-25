import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    os.environ["XDG_SESSION_TYPE"] = "xcb"
    
    return LaunchDescription([
        # Declare launch arguments (optional)
        DeclareLaunchArgument('use_sim_time', default_value='false', 
                               description='Use simulation time if true'),

        Node(
            package='brain',
            executable='brain',  # Replace with the actual executable name
            name='brain',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Include other launch files from different packages
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('vision'), 'launch'),
                '/vision_launch.py'])
            ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('end_effector'), 'launch'),
                '/end_effector_launch.py'])
            ),

        # # Launch RViz
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        # ),
    ])