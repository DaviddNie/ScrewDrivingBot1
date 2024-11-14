# Return a list of nodes we commonly launch for the demo. Nice for testing use.
import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


# def get_robot_description():
#     robot_description_config = xacro.process_file(
#         os.path.join(
#             get_package_share_directory("moveit_resources_panda_moveit_config"),
#             "config",
#             "panda.urdf.xacro",
#         )
#     )
#     robot_description = {"robot_description": robot_description_config.toxml()}
#     return robot_description


# def get_robot_description_semantic():
#     robot_description_semantic_config = load_file(
#         "moveit_resources_panda_moveit_config", "config/panda.srdf"
#     )
#     robot_description_semantic = {
#         "robot_description_semantic": robot_description_semantic_config
#     }
#     return robot_description_semantic

def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            # PathJoinSubstitution([FindPackageShare("end_effector_description"), "urdf", "end_effector_withDriverSupport.xacro"]),
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=172.17.0.2",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
           "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur5e",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )


    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            "ur",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic


def generate_common_hybrid_launch_description():
    robot_description = get_robot_description()

    robot_description_semantic = get_robot_description_semantic()

    # The global planner uses the typical OMPL parameters
    planning_pipelines_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    
    ompl_planning_yaml = load_yaml(
        "ur_moveit_config", "config/ompl_planning.yaml"
    )
    planning_pipelines_config["ompl"].update(ompl_planning_yaml)

    kinematics_param = load_yaml(
        "ur_moveit_config", "config/kinematics.yaml")

    moveit_controllers_yaml = load_yaml(
        "ur_moveit_config", "config/controllers.yaml")
    
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Any parameters that are unique to your plugins go here
    common_hybrid_planning_param = load_yaml(
        "moveit_hybrid_planning", "config/common_hybrid_planning_params.yaml"
    )
    global_planner_param = load_yaml(
        "moveit_hybrid_planning", "config/global_planner.yaml"
    )
    local_planner_param = load_yaml(
        "moveit_hybrid_planning", "config/local_planner.yaml"
    )
    hybrid_planning_manager_param = load_yaml(
        "moveit_hybrid_planning", "config/hybrid_planning_manager.yaml"
    )

    # Generate launch description with multiple components
    container = ComposableNodeContainer(
        name="hybrid_planning_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner",
                parameters=[
                    common_hybrid_planning_param,
                    global_planner_param,
                    robot_description,
                    robot_description_semantic,
                    planning_pipelines_config,
                    moveit_controllers,
                    kinematics_param
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner",
                parameters=[
                    common_hybrid_planning_param,
                    local_planner_param,
                    robot_description,
                    robot_description_semantic,
                    kinematics_param
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                parameters=[
                    common_hybrid_planning_param,
                    hybrid_planning_manager_param,
                    kinematics_param
                ],
            ),
        ],
        output="screen",
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_hybrid_planning")
        + "/config/hybrid_planning_demo.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description, robot_description_semantic],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    ur_joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen"
    )

    def get_ur_control_launch():
        """Configure UR control launch for the UR5e arm."""
        # Toggle between simulated or real UR5e hardware
        use_fake = True
        use_fake_str = 'true'
        ur_type = 'ur5e'
        ip_address = 'yyy.yyy.yyy.yyy'

        if not use_fake:
            ip_address = '192.168.0.100'
            use_fake_str = 'false'
        
        end_effector_path = os.path.join(
            get_package_share_directory('end_effector_description'), 'urdf', 'end_effector_withDriverSupport.xacro'
        )

        ur_control_launch_args = {
            'ur_type': ur_type,
            'robot_ip': ip_address,
            'use_fake_hardware': use_fake_str,
            'launch_rviz': 'false',  
            'description_file': end_effector_path,
        }

        # Add controller if using simulated hardware
        if use_fake:
            ur_control_launch_args['initial_joint_controller'] = 'joint_trajectory_controller'

        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])
            ),
            launch_arguments=ur_control_launch_args.items(),
        )


    launched_nodes = [
        container,
        static_tf,
        rviz_node,
        robot_state_publisher,
        get_ur_control_launch(),
        joint_state_broadcaster_spawner,
        ur_joint_trajectory_controller_spawner,
    ]

    return launched_nodes
