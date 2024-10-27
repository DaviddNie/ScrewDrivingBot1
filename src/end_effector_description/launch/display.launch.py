import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
import xacro
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name = 'end_effector_description'
    xacro_path = 'urdf/end_effector_withDriverSupport.xacro'
    rviz_path = 'rviz/view_robot.rviz'
    
    use_fake = True
    ip_address = 'yyy.yyy.yyy.yyy'
    use_fake_str = 'true'

    if not use_fake:
        ip_address = '192.168.0.100'
        use_fake_str = 'false'
	

    xacro_file = os.path.join(get_package_share_directory(package_name), xacro_path)
    xacro_raw_description = xacro.process_file(xacro_file).toxml()

    rviz_file = os.path.join(get_package_share_directory(package_name), rviz_path)

    ur_control_launch_args = {
        'ur_type': 'ur5e',
        'robot_ip': ip_address,
        'use_fake_hardware': use_fake_str,
        'launch_rviz': 'false',
        'debug': 'true',
        'description_file': '/home/davidnie/4231/ScrewDrivingBot1/install/end_effector_description/share/end_effector_description/urdf/end_effector_withDriverSupport.xacro'
    }

    moveit_launch_args = {
        'ur_type': 'ur5e',
        'launch_rviz': 'true',
    }

    if use_fake:
        ur_control_launch_args['initial_joint_controller'] = 'joint_trajectory_controller'
        moveit_launch_args['use_fake_hardware'] = use_fake_str


    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'
            ])
        ),
        launch_arguments=ur_control_launch_args.items(),
    )

    # Define the MoveIt server launch with a delay
    moveit_launch = TimerAction(
        period=10.0,  # Delay to allow the UR control to start first
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py'
                    ])
                ),
                launch_arguments=moveit_launch_args.items(),
            ),
        ]
    )

    robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': xacro_raw_description}])

    joint_state_publisher = Node(
            name='joint_state_publisher_gui',
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': xacro_raw_description}])
    
    rviz_launch = Node(
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
			)
    
    launch_description = [
        robot_state_publisher,
        ur_control_launch,
        moveit_launch,

        # The UR5e glitches when enabled
    ]

    # The UR5e glitches when enabled
    # if use_fake:
    #     launch_description.append(joint_state_publisher)
    
    # launch_description.append(rviz_launch)

    return LaunchDescription(launch_description)