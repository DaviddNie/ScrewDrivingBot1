#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <vector>
#include <sstream>
#include <cmath>

// Constants for Cartesian path toggle and tolerance values
constexpr double PLANNING_TIME = 20.0;
constexpr int PLANNING_ATTEMPTS = 15;
constexpr double GOAL_TOLERANCE = 0.01;

// Structure for joint constraint configuration
struct JointConstraintConfig {
    std::string joint_name; double position; double tolerance_above; double tolerance_below;
};

// Joint constraints for each joint
const std::vector<JointConstraintConfig> JOINT_CONSTRAINTS = {
    { "shoulder_pan_joint",  0,  M_PI / 3,  M_PI / 3 },
    { "shoulder_lift_joint",  -M_PI / 2,  M_PI /2,  M_PI /2},
    // { "elbow_joint",          0,  M_PI,  M_PI },
    { "wrist_1_joint",           M_PI/2,      M_PI*4/5,      M_PI*4/5},
    { "wrist_2_joint",              0,  M_PI/2,  M_PI/2 },
    { "wrist_3_joint",        M_PI / 2,  M_PI / 2,  M_PI / 2 }
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

        move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
        move_group_interface_->setPlanningTime(PLANNING_TIME);
        move_group_interface_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
        move_group_interface_->setGoalTolerance(GOAL_TOLERANCE);

        setupCollisionObjects();

        RCLCPP_INFO(this->get_logger(), "ArmMovement node initialized.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr movement_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr movement_done_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr movement_status_pub_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

    void movementCallback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received movement command: %s", msg->data.c_str());
        std::stringstream ss;
        ss << "Received command: " << msg->data;
        publishArmStatus(ss.str());

        if (msg->data == "home") {
            moveToHome();
        } else if (msg->data.rfind("hole", 0) == 0) {
            processHoleCommand(msg->data);
        } else if (msg->data == "tool") {
            moveToTool();
        }
    }

    void setupCollisionObjects() {
        // std::string frame_id = move_group_interface_->getPlanningFrame();
        std::string frame_id = "world";
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall"));
        planning_scene_interface.applyCollisionObject(generateCollisionObject(0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall"));
        planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 0.05, frame_id, "table"));
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

    void setupLineConstraint(moveit_msgs::msg::Constraints& constraints) {
        moveit_msgs::msg::PositionConstraint line_constraint;
        line_constraint.header.frame_id = "base_link";  // Reference frame for the constraint
        line_constraint.link_name = "4231_tool_link";

        shape_msgs::msg::SolidPrimitive line;
        line.type = shape_msgs::msg::SolidPrimitive::BOX;
        line.dimensions = { 0.0005, 0.0005, 1.0 };  // X,Y,Z
        line_constraint.constraint_region.primitives.emplace_back(line);

        // Define the box position offset to indicate the starting point along the Z-axis
        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = 0.0;  // Centered along X-axis
        box_pose.position.y = 0.0;  // Centered along Y-axis
        box_pose.position.z = 0.5;  // Starting Z position (modify as needed)

        line_constraint.constraint_region.primitive_poses.push_back(box_pose);
        line_constraint.weight = 1.0;

        // Add this line constraint to the position constraints
        constraints.position_constraints.push_back(line_constraint);

        geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose().pose;
        geometry_msgs::msg::Pose line_pose;
        line_pose.position.x = current_pose.position.x;
        line_pose.position.y = current_pose.position.y;
        line_pose.position.z = current_pose.position.z;
        line_pose.orientation = DEFAULT_ORIENTATION;
        line_constraint.weight = 1.0;
        line_constraint.constraint_region.primitive_poses.emplace_back(line_pose);
    }


    void moveToHome() {
        RCLCPP_INFO(this->get_logger(), "Moving to home position.");
        publishArmStatus("moving to home");
        geometry_msgs::msg::Pose home_pose;
        home_pose.position.x = 0.35;
        home_pose.position.y = 0.35;
        home_pose.position.z = 0.50;
        home_pose.orientation = DEFAULT_ORIENTATION;
        moveToPose(home_pose, "joint");
    }

    void processHoleCommand(const std::string& data) {
        double x, y, z;
        if (parseCoordinates(data, x, y, z)) {
            std::stringstream ss;
            ss << "Moving to hole position at x: " << x << ", y: " << y << ", z: " << z;
            publishArmStatus(ss.str());

            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = x;
            target_pose.position.y = y;
            target_pose.position.z = z;
            target_pose.orientation = DEFAULT_ORIENTATION;
            moveToPose(target_pose, "line");
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

    void moveToTool() {
        RCLCPP_INFO(this->get_logger(), "Moving to tool position.");
        publishArmStatus("moving to tool");
        geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_pose.position.x + 0.620;
        target_pose.position.y = current_pose.position.y;
        target_pose.position.z = 0.3;
        target_pose.orientation = DEFAULT_ORIENTATION;
        moveToPose(target_pose, "line");
    }

    void moveToPose(const geometry_msgs::msg::Pose &pose, const std::string &constraintType) {
        moveit_msgs::msg::Constraints constraints;

        if (constraintType == "joint") {
            RCLCPP_INFO(this->get_logger(), "Using joint constraints.");
            setupJointConstraints(constraints);  // Adds joint constraints
        } else if (constraintType == "line") {
            RCLCPP_INFO(this->get_logger(), "Using line constraint along Z-axis.");
            setupLineConstraint(constraints);  // Adds line constraint
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown constraint type. Using default joint constraints.");
            setupJointConstraints(constraints);  // Default to joint constraints
        }

        move_group_interface_->setPoseTarget(pose);
        move_group_interface_->setPathConstraints(constraints);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Executing move.");
            publishArmStatus("executing now");
            move_group_interface_->execute(plan);
            move_group_interface_->stop();
            publishArmStatus("movement done");
            publishArmDone("done");
        } else {
            RCLCPP_WARN(this->get_logger(), "Path planning failed.");
            publishArmStatus("planninng failed, movement failed");
            publishArmDone("fail");
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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmMovement>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
