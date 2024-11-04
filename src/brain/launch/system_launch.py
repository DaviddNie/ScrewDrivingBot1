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
    
    
    brain_node = Node(
            package='brain',
            executable='brain',  # Replace with the actual executable name
            name='brain',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    
    robotAndCamera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('end_effector_description'), 'launch'),
                '/display.launch.py'])
            )
    vision = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('vision'), 'launch'),
                '/vision_launch.py'])
            )
    
    end_effector = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('end_effector'), 'launch'),
                '/end_effector_launch.py'])
            )
    
    transformations = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('transformations'), 'launch'),
                '/transformations_launch.py'])
            )
    
    arm = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('movement'), 'launch'),
                '/arm_launch.py'])
            )
    return LaunchDescription([
        # Declare launch arguments (optional)
        DeclareLaunchArgument('use_sim_time', default_value='false', 
                               description='Use simulation time if true'),

        brain_node,
        robotAndCamera,
        vision,
        end_effector,
        arm,
        transformations,
    ])