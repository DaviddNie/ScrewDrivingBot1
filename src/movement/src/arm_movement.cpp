#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <vector>
#include <sstream>
#include <cmath>

// Constants for Cartesian path toggle and tolerance values
constexpr double PLANNING_TIME = 20.0;
constexpr int PLANNING_ATTEMPTS = 15;
constexpr double GOAL_TOLERANCE = 0.00005; // 0.1 mm, 0.0001 m

// Structure for joint constraint configuration
struct JointConstraintConfig {
    std::string joint_name; double position; double tolerance_above; double tolerance_below;
};

// Joint constraints for each joint
const std::vector<JointConstraintConfig> JOINT_CONSTRAINTS = {
    { "shoulder_pan_joint",  0,  M_PI / 2,  M_PI / 2 },
    { "shoulder_lift_joint",  -M_PI / 2,  M_PI /2,  M_PI /2},
    // { "elbow_joint",          0,  M_PI,  M_PI },
    { "wrist_1_joint",           M_PI/2,      M_PI*4/5,      M_PI*4/5},
    { "wrist_2_joint",              0,  M_PI/1.5,  M_PI/1.5 },
    { "wrist_3_joint",        M_PI / 2,  M_PI / 1.5,  M_PI / 1.5 }
};

// Default orientation for the end effector
const geometry_msgs::msg::Quaternion DEFAULT_ORIENTATION = [] {
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = 0.5;
    orientation.y = 0.5;
    orientation.z = 0.5;
    orientation.w = -0.5;
    return orientation;
}();

class ArmMovement : public rclcpp::Node {
public:
    ArmMovement() : Node("arm_movement") {
        movement_sub_ = this->create_subscription<std_msgs::msg::String>("move_to", 10, std::bind(&ArmMovement::movementCallback, this, std::placeholders::_1));
        movement_done_pub_ = this->create_publisher<std_msgs::msg::String>("done_move", 10);
        movement_status_pub_ = this->create_publisher<std_msgs::msg::String>("arm_status", 10);
        text_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
        move_group_interface_->setPlannerId("LIN");
        move_group_interface_->setPlanningTime(PLANNING_TIME);
        move_group_interface_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
        move_group_interface_->setGoalTolerance(GOAL_TOLERANCE);
        move_group_interface_->setStartStateToCurrentState();

        setupCollisionObjects();

        timer_tool0_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { this->publishToolPositionText("tool0"); }
        );

        moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            shared_from_this(), "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface_->getRobotModel()
        );

        moveit_visual_tools_->deleteAllMarkers();
        moveit_visual_tools_->loadRemoteControl();

        RCLCPP_INFO(this->get_logger(), "ArmMovement node initialized.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr movement_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr movement_done_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr movement_status_pub_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr text_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_tool0_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;

    void movementCallback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received movement command: %s", msg->data.c_str());
        std::stringstream ss;
        ss << "Received command: " << msg->data;
        publishArmStatus(ss.str());

        if (msg->data == "home") {
            moveToHome();
        } else if (msg->data.rfind("hole", 0) == 0) {
            moveToHole(msg->data);
        } else if (msg->data.rfind("tool", 0) == 0) {
            moveToTool(msg->data);
        }
    }

    void setupCollisionObjects() {
        std::string frame_id = "world";
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall"));
        planning_scene_interface.applyCollisionObject(generateCollisionObject(0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall"));
        planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.01, 0.85, 0.25, 0.013, frame_id, "table"));
        planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 1.2, frame_id, "ceiling"));
    }

    auto generateCollisionObject(float sx, float sy, float sz, float x, float y, float z, const std::string& frame_id, const std::string& id) -> moveit_msgs::msg::CollisionObject {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = id;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {sx, sy, sz};

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = x;
        box_pose.position.y = y;
        box_pose.position.z = z;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

    return collision_object;
}

    void setupJointConstraints(moveit_msgs::msg::Constraints &constraints) {
        for (const auto& config : JOINT_CONSTRAINTS) {
            moveit_msgs::msg::JointConstraint constraint;
            constraint.joint_name = config.joint_name;
            constraint.position = config.position;
            constraint.tolerance_above = config.tolerance_above;
            constraint.tolerance_below = config.tolerance_below;
            constraint.weight = 1.0;
            constraints.joint_constraints.push_back(constraint);
        }
    }

    void setupLineConstraint(moveit_msgs::msg::Constraints& constraints, const geometry_msgs::msg::Pose& target_pose) {
        moveit_msgs::msg::PositionConstraint line_constraint;
        line_constraint.header.frame_id = "base_link";  // Reference frame for the constraint
        line_constraint.link_name = move_group_interface_->getEndEffectorLink(); // "tool0"; End effector link for the ur5e

        RCLCPP_INFO(this->get_logger(), "Header frame: %s", line_constraint.header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", line_constraint.link_name.c_str());

        shape_msgs::msg::SolidPrimitive line;
        line.type = shape_msgs::msg::SolidPrimitive::BOX;
        line.dimensions = { 0.1, 0.1, 1.0 };  // X,Y,Z
        line_constraint.constraint_region.primitives.emplace_back(line);

        auto current_pose = move_group_interface_->getCurrentPose("tool0").pose;
        geometry_msgs::msg::Pose line_pose;
        line_pose.position.x = current_pose.position.x;
        line_pose.position.y = current_pose.position.y;
        line_pose.position.z = current_pose.position.z;
        line_pose.orientation = DEFAULT_ORIENTATION;
        
        line_constraint.constraint_region.primitive_poses.emplace_back(line_pose);
        line_constraint.weight = 1.0;

        // Visualize the constraint
        moveit_visual_tools_->publishLine(current_pose.position, target_pose.position,
                                        rviz_visual_tools::TRANSLUCENT_DARK);
        moveit_visual_tools_->trigger();

        constraints.position_constraints.emplace_back(line_constraint);
        constraints.name = "use_equality_constraints";
    }



    void moveToHome() {
        RCLCPP_INFO(this->get_logger(), "Moving to home position.");
        publishArmStatus("moving to home");
        geometry_msgs::msg::Pose home_pose;
        home_pose.position.x = 0.4;
        home_pose.position.y = 0.4;
        home_pose.position.z = 0.30;
        home_pose.orientation = DEFAULT_ORIENTATION;
        moveToPose(home_pose, "joint");
    }

    void moveToHole(const std::string& data) {
        std::stringstream ss;
        ss << "Moving to hole position at x: " << data;
        publishArmStatus(ss.str());
        double x, y, z;
        if (parseCoordinates(data, x, y, z)) {
            std::stringstream ss;
            ss << "Moving to hole position at x: " << x << ", y: " << y << ", z: " << z;
            publishArmStatus(ss.str());
          
            // shift from tool0 to tool point
            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = x;
            target_pose.position.y = y - 0.03013;
            target_pose.position.z = z + 0.097457;
            target_pose.orientation = DEFAULT_ORIENTATION;
            moveToPose(target_pose, "joint");
        }
    }

    bool parseCoordinates(const std::string& data, double& x, double& y, double& z) {
        std::vector<std::string> tokens;
        std::stringstream ss(data);
        std::string token;

        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        if (tokens.size() == 4) {
            x = std::stod(tokens[1]);
            y = std::stod(tokens[2]);
            z = std::stod(tokens[3]);
            return true;
        }
        return false;
    }

    void moveToTool(const std::string& data) {
        RCLCPP_INFO(this->get_logger(), "Recieved tool request.");

        double x, y, z;
        if (parseCoordinates(data, x, y, z)) {
            RCLCPP_INFO(this->get_logger(), "Moving to tool position.");
            publishArmStatus("moving to tool");
            auto current_pose = move_group_interface_->getCurrentPose("tool0").pose;

            std::stringstream ss;
            ss << "Moving tool to hole position at x: " << current_pose.position.x << ", y: " << current_pose.position.y << ", z: " << current_pose.position.z+z;
            publishArmStatus(ss.str());

            geometry_msgs::msg::Pose target_pose;
            target_pose = current_pose;
            target_pose.position.z += z;

            double speed = 0.001;
            moveToPose(target_pose, "cartesian", speed);
        }
    }

    bool executeCartesianPath(const geometry_msgs::msg::Pose &start_pose, const geometry_msgs::msg::Pose &target_pose, double speed) {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(start_pose);  // Start from current pose
        waypoints.push_back(target_pose); // Move to target pose

        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.00001; 
        double jump_threshold = 0.0;
        const double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.9) {  // Check if a sufficient fraction of the path was planned
            RCLCPP_INFO(this->get_logger(), "Cartesian path computed successfully with %.2f%% of the path", fraction * 100.0);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;

            RCLCPP_INFO(this->get_logger(), "Set Max. Velocity: %.2f", speed);
            move_group_interface_->setMaxVelocityScalingFactor(speed);
            move_group_interface_->setMaxAccelerationScalingFactor(speed);

            move_group_interface_->execute(plan);

            publishArmStatus("cartesian movement done");
            publishArmDone("done");
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Cartesian path planning failed with only %.2f%% of the path", fraction * 100.0);
            publishArmStatus("cartesian movement failed");
            publishArmDone("fail");
            return false;
        }
    }

    void moveToPose(const geometry_msgs::msg::Pose &target_pose, const std::string &constraintType, double speed = 0.3) {
        move_group_interface_->clearPathConstraints();

        auto current_pose = move_group_interface_->getCurrentPose("tool0").pose;
        moveit_visual_tools_->deleteAllMarkers();
        moveit_visual_tools_->publishSphere(current_pose, rviz_visual_tools::RED, 0.05);
        moveit_visual_tools_->publishSphere(target_pose, rviz_visual_tools::GREEN, 0.05);
        moveit_visual_tools_->trigger();

        if (constraintType == "cartesian") {
            // Execute Cartesian path planning and execution
            if (!executeCartesianPath(current_pose, target_pose, speed)) {
                RCLCPP_WARN(this->get_logger(), "Cartesian path execution failed.");
            }
        } else {
            moveit_msgs::msg::Constraints constraints;

            if (constraintType == "joint") {
                RCLCPP_INFO(this->get_logger(), "Using joint constraints.");
                setupJointConstraints(constraints);  // Adds joint constraints
            } else if (constraintType == "line") {
                RCLCPP_INFO(this->get_logger(), "Using line constraint along Z-axis.");
                setupLineConstraint(constraints, target_pose);  // Adds line constraint
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown constraint type. Using default joint constraints.");
                setupJointConstraints(constraints);  // Default to joint constraints
            }

            move_group_interface_->setPoseTarget(target_pose);
            move_group_interface_->setPathConstraints(constraints);

            RCLCPP_INFO(this->get_logger(), "Set Max. Velocity: %.2f", speed);
            move_group_interface_->setMaxVelocityScalingFactor(speed);
            move_group_interface_->setMaxAccelerationScalingFactor(speed);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Executing move.");
                publishArmStatus("executing now");
                publishTargetMarker(target_pose);  // Show target marker
                move_group_interface_->execute(plan);
                // move_group_interface_->stop();
                publishArmStatus("movement done");
                publishArmDone("done");
            } else {
                RCLCPP_WARN(this->get_logger(), "Path planning failed.");
                publishArmStatus("planninng failed, movement failed");
                publishArmDone("fail");
            }
        }

        move_group_interface_->clearPathConstraints();
    }

    void publishArmDone(const std::string &done)
    {
        std_msgs::msg::String done_msg;
        done_msg.data = done;
        movement_done_pub_->publish(done_msg);
        RCLCPP_INFO(this->get_logger(), "Published arm done: %s", done.c_str());
    }

    void publishArmStatus(const std::string &status) {
        std_msgs::msg::String status_msg;
        status_msg.data = status;
        movement_status_pub_->publish(status_msg);
        RCLCPP_INFO(this->get_logger(), "Published arm status: %s", status.c_str());
    }

    void publishTargetMarker(const geometry_msgs::msg::Pose &target_pose) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";  // Use the correct frame
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "target_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = target_pose;
        marker.scale.x = 0.05; 
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;   
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        text_marker_pub_->publish(marker);
    }

    void publishToolPositionText(const std::string &link_name) {
        auto tool_pose = move_group_interface_->getCurrentPose(link_name).pose;
        
        // Prepare a text marker to display the position
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "base_link";
        text_marker.header.stamp = this->get_clock()->now();
        text_marker.ns = "tool_position";
        text_marker.id = 0;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        // Position the text near the tool
        text_marker.pose.position.x = tool_pose.position.x;
        text_marker.pose.position.y = tool_pose.position.y;
        text_marker.pose.position.z = tool_pose.position.z + 0.1;  // Offset above tool0 position
        text_marker.pose.orientation.w = 1.0;

        // Customize the text and appearance
        std::stringstream ss;
        ss << "X: " << tool_pose.position.x << "\nY: " << tool_pose.position.y << "\nZ: " << tool_pose.position.z;
        text_marker.text = ss.str();
        text_marker.scale.z = 0.05;  // Text height
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        // Publish the text marker
        text_marker_pub_->publish(text_marker);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmMovement>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
