#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <vector>
#include <sstream>
#include <cmath>
#include <thread>
#include <tf2_ros/static_transform_broadcaster.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/create_client.hpp>
#include "interfaces/srv/hybrid_cmd.hpp"

using namespace std::chrono_literals;
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("test_hybrid_planning_client");
}  // namespace

class HybridPlanningTest : public rclcpp::Node
{
public:
  HybridPlanningTest() : Node("hybrid_planning_test")
  {
    // Initialize Hybrid planning action client
    this->declare_parameter<std::string>("hybrid_planning_action_name", "/test/hybrid_planning/run_hybrid_planning");

    std::string hybrid_planning_action_name = "";
    if (this->has_parameter("hybrid_planning_action_name"))
    {
      this->get_parameter<std::string>("hybrid_planning_action_name", hybrid_planning_action_name);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "hybrid_planning_action_name parameter was not defined");
      std::exit(EXIT_FAILURE);
    }
    hp_action_client_ = rclcpp_action::create_client<moveit_msgs::action::HybridPlanner>(this, hybrid_planning_action_name);
        if (!hp_action_client_->wait_for_action_server(20s))
    {
      RCLCPP_ERROR(this->get_logger(), "Hybrid planning action server not available after waiting");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Hybrid planning client initialized");

    

    // Create service to set goal pose
    goal_service_ = this->create_service<interfaces::srv::HybridCmd>(
      "set_goal_pose",
      std::bind(&HybridPlanningTest::handleGoalPoseRequest, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "Service 'set_goal_pose' ready for goal requests.");
  }

  void initializePlanningSceneMonitor()
    {
        // Initialize Planning Scene Monitor
        RCLCPP_INFO(this->get_logger(), "Initialize Planning Scene Monitor");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            shared_from_this(), "robot_description", tf_buffer_, "planning_scene_monitor");

        if (!planning_scene_monitor_->getPlanningScene())
        {
          RCLCPP_ERROR(LOGGER, "The planning scene was not retrieved!");
          return;
        }
        else
        {
          planning_scene_monitor_->startStateMonitor();
          planning_scene_monitor_->providePlanningSceneService();  // Allows RViz to query the PlanningScene
          planning_scene_monitor_->setPlanningScenePublishingFrequency(100);
          planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "/planning_scene");
          planning_scene_monitor_->startSceneMonitor();
        }

        if (!planning_scene_monitor_->waitForCurrentRobotState(this->now(), 5))
        {
          RCLCPP_ERROR(this->get_logger(), "Timeout when waiting for /joint_states updates. Is the robot running?");
          return;
        }

        if (!hp_action_client_->wait_for_action_server(20s))
        {
          RCLCPP_ERROR(LOGGER, "Hybrid planning action server not available after waiting");
          return;
        }
    }

private:
    rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SharedPtr hp_action_client_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Service<interfaces::srv::HybridCmd>::SharedPtr goal_service_;


    void handleGoalPoseRequest(const std::shared_ptr<interfaces::srv::HybridCmd::Request> request,
                           std::shared_ptr<interfaces::srv::HybridCmd::Response> response)
    { 
        RCLCPP_INFO(this->get_logger(), "Received movement request:");
     
        // Setup motion planning goal taken from motion_planning_api tutorial
        const std::string planning_group = "ur_manipulator";
        robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
        const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();

        // Create a RobotState and JointModelGroup
        const auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
        const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group);

        // Configure a valid robot state
        robot_state->setToDefaultValues(joint_model_group, "ready");
        robot_state->update();
        // Lock the planning scene as briefly as possible
        {
          planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);
          locked_planning_scene->setCurrentState(*robot_state);
        }

        // // Define target pose from request data
        // geometry_msgs::msg::Pose target_pose;
        // target_pose.position.x = request->x;
        // target_pose.position.y = request->y;
        // target_pose.position.z = request->z;
        // // target_pose.orientation.w = 1.0; // Use this or request orientation as needed


        //  // Perform IK to get joint positions
        // moveit::core::RobotState goal_state(robot_model);
        // bool found_ik = goal_state.setFromIK(joint_model_group, target_pose);

        // if (!found_ik)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "IK solution not found for target pose");
        //     response->success = false;
        //     return;
        // }

        // Create desired motion goal
        moveit_msgs::msg::MotionPlanRequest goal_motion_request;
        moveit::core::robotStateToRobotStateMsg(*robot_state, goal_motion_request.start_state);
        goal_motion_request.group_name = planning_group;
        goal_motion_request.num_planning_attempts = 10;
        goal_motion_request.max_velocity_scaling_factor = 0.1;
        goal_motion_request.max_acceleration_scaling_factor = 0.1;
        goal_motion_request.allowed_planning_time = 2.0;
        goal_motion_request.planner_id = "ompl";
        goal_motion_request.pipeline_id = "ompl";

        moveit::core::RobotState goal_state(robot_model);
        // Extract requested pose
        std::vector<double> joint_values = {
          request->shoulder_pan_joint,
          request->shoulder_lift_joint,
          request->elbow_joint,
          request->wrist_1_joint,
          request->wrist_2_joint,
          request->wrist_3_joint,
        };
        goal_state.setJointGroupPositions(joint_model_group, joint_values);
        
        goal_motion_request.goal_constraints.resize(1);
        goal_motion_request.goal_constraints[0] =
            kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

        // Create Hybrid Planning action request
        moveit_msgs::msg::MotionSequenceItem sequence_item;
        sequence_item.req = goal_motion_request;
        sequence_item.blend_radius = 0.0;  // Single goal

        moveit_msgs::msg::MotionSequenceRequest sequence_request;
        sequence_request.items.push_back(sequence_item);

        auto goal_action_request = moveit_msgs::action::HybridPlanner::Goal();
        goal_action_request.planning_group = planning_group;
        goal_action_request.motion_sequence = sequence_request;

        auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SendGoalOptions();
        send_goal_options.result_callback =
            [](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::WrappedResult& result) {
              switch (result.code)
              {
                case rclcpp_action::ResultCode::SUCCEEDED:
                  RCLCPP_INFO(LOGGER, "Hybrid planning goal succeeded");
                  break;
                case rclcpp_action::ResultCode::ABORTED:
                  RCLCPP_ERROR(LOGGER, "Hybrid planning goal was aborted");
                  return;
                case rclcpp_action::ResultCode::CANCELED:
                  RCLCPP_ERROR(LOGGER, "Hybrid planning goal was canceled");
                  return;
                default:
                  RCLCPP_ERROR(LOGGER, "Unknown hybrid planning result code");
                  return;
                  RCLCPP_INFO(LOGGER, "Hybrid planning result received");
              }
            };
        send_goal_options.feedback_callback =
            [](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::SharedPtr& /*unused*/,
              const std::shared_ptr<const moveit_msgs::action::HybridPlanner::Feedback>& feedback) {
              RCLCPP_INFO_STREAM(LOGGER, feedback->feedback);
            };

        RCLCPP_INFO(LOGGER, "Sending hybrid planning goal");
        // Ask server to achieve some goal and wait until it's accepted
        auto goal_handle_future = hp_action_client_->async_send_goal(goal_action_request, send_goal_options);
        response->success = true;
      }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HybridPlanningTest>();
    node->initializePlanningSceneMonitor();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
