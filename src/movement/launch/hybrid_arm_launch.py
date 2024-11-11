import launch
import os
import sys
import yaml

from launch_ros.actions import Node, ComposableNodeContainer
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

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
            PathJoinSubstitution([FindPackageShare("end_effector_description"), "urdf", "end_effector_withDriverSupport.xacro"]),
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

def get_hybrid_planning_container(robot_description, robot_description_semantic):
    # Load configurations for hybrid planning components
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
    kinematics_yaml = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]
    )

    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    return ComposableNodeContainer(
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
                    ompl_planning_pipeline_config,
                    kinematics_yaml
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
                    kinematics_yaml
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                parameters=[
                    common_hybrid_planning_param,
                    hybrid_planning_manager_param,
                ],
            ),
        ],
        output="screen",
    )

    
def generate_launch_description():
    common_hybrid_planning_param = load_yaml(
        "moveit_hybrid_planning", "config/common_hybrid_planning_params.yaml"
    )
    
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    arm_node = Node(
        package="movement",
        executable="arm_movement",
        name="arm_movement",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            common_hybrid_planning_param,
            {"use_sim_time": True},
        ],
    )
    arm_brain_node = Node(
        package="movement",
        executable="arm_brain",
        name="arm_brain",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            common_hybrid_planning_param,
        ],
    )
    demo_node = Node(
        package="movement",
        executable="hybrid_planning_demo_node",
        name="hybrid_planning_demo_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            common_hybrid_planning_param,
        ],
    )

   
    hybrid_planning_container = get_hybrid_planning_container(robot_description, robot_description_semantic)

    return launch.LaunchDescription([demo_node])