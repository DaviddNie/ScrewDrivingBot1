#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

auto generateCollisionObject(float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id) {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = sx;
  primitive.dimensions[primitive.BOX_Y] = sy;
  primitive.dimensions[primitive.BOX_Z] = sz;

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

class ArmMovement : public rclcpp::Node
{
public:
    ArmMovement() : Node("arm_movement")
    {
        // Subscribe to movement commands
        movement_sub_ = this->create_subscription<std_msgs::msg::String>(
            "move_to", 10, std::bind(&ArmMovement::movementCallback, this, std::placeholders::_1));

        // Publisher to notify when movement is done
        movement_done_pub_ = this->create_publisher<std_msgs::msg::String>("done_move", 10);

        // Initialize MoveIt move group interface
        move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");

        move_group_interface_->setPlanningTime(10.0);
        move_group_interface_->setNumPlanningAttempts(10);
        std::string frame_id = move_group_interface_->getPlanningFrame();

        auto col_object_backWall = generateCollisionObject( 2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall");
        auto col_object_sideWall = generateCollisionObject( 0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall");
        auto col_object_table = generateCollisionObject( 2.4, 2.4, 0.04, 0.85, 0.25, 0.05, frame_id, "table");

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        // Apply table as a collision object
        planning_scene_interface.applyCollisionObject(col_object_backWall);
        planning_scene_interface.applyCollisionObject(col_object_sideWall);
        planning_scene_interface.applyCollisionObject(col_object_table);

        moveit_msgs::msg::JointConstraint wrist1;
        wrist1.joint_name = "wrist_1_joint";  // Adjust the joint controlling the end-effector orientation
        wrist1.position = 0.0;  // Keep the wrist joint in its default position
        wrist1.tolerance_above = M_PI/2;
        wrist1.tolerance_below = M_PI/2;
        wrist1.weight = 1.0;

        moveit_msgs::msg::JointConstraint elbow;
        elbow.joint_name = "elbow";
        elbow.position = 0.0;  // Keep the wrist fixed in the desired orientation
        elbow.tolerance_above = M_PI/2;
        elbow.tolerance_below = M_PI/2;
        elbow.weight = 1.0;

        joint_constraints_.joint_constraints.push_back(wrist1);
        joint_constraints_.joint_constraints.push_back(elbow);

        // moveit_msgs::msg::OrientationConstraint ocm;
        // ocm.link_name = move_group_interface_->getEndEffectorLink(); // Set for the wrist link
        // ocm.header.frame_id = move_group_interface_->getPoseReferenceFrame();
        // ocm.orientation.x = 0.66;
        // ocm.orientation.y = 0.24;
        // ocm.orientation.z = 0.67;
        // ocm.orientation.w = -0.23;  // Ensure end-effector remains horizontal
        // ocm.absolute_x_axis_tolerance = M_PI/2;
        // ocm.absolute_y_axis_tolerance = M_PI/2;
        // ocm.absolute_z_axis_tolerance = M_PI/2; // Allow full rotation around z-axis
        // ocm.weight = 1.0;

        // orientation_constraints_.orientation_constraints.push_back(ocm);

        RCLCPP_INFO(this->get_logger(), "ArmMovement node initialized.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr movement_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr movement_done_pub_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    moveit_msgs::msg::Constraints joint_constraints_;

    // Callback for movement commands
    void movementCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received movement command: %s", msg->data.c_str());

        if (msg->data == "home") {
            moveToHome();
        } else if (msg->data.rfind("hole", 0) == 0) {
            // Parse hole coordinates from message (e.g., "hole,x,y")
            std::vector<std::string> tokens;
            std::stringstream ss(msg->data);
            std::string token;
            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }

            if (tokens.size() == 4) {
                double x = std::stod(tokens[1]);
                double y = std::stod(tokens[2]);
                double z = std::stod(tokens[3]);
                RCLCPP_INFO(this->get_logger(), "Parsed hole coordinates: x=%f, y=%f, z=%f", x, y, z);
                moveToHole(x, y, z);
            }
        } else if (msg->data == "tool") {
            moveToTool();
        }

        // // Notify movement done
        // std_msgs::msg::String done_msg;
        // done_msg.data = "done";
        // movement_done_pub_->publish(done_msg);
    }

    void publishArmStatus(const std::string &status)
    {
        // Create a message object
        std_msgs::msg::String status_msg;
        
        // Set the status message ("done" or "fail")
        status_msg.data = status;

        // Publish the status
        movement_done_pub_->publish(status_msg);

        // Log the status
        RCLCPP_INFO(this->get_logger(), "Published arm status: %s", status.c_str());
    }

    // Move to home position (vertically first, then to home)
    void moveToHome()
    {
        RCLCPP_INFO(this->get_logger(), "Moving to home position.");

        // geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose().pose;
        // geometry_msgs::msg::Pose vertical_pose = current_pose;
        // vertical_pose.position.z = 0.620;  // Move to a safe z position (e.g., above obstacles)
        // moveToPose(vertical_pose);

        geometry_msgs::msg::Pose home_pose;
        home_pose.position.x = 0.436;
        home_pose.position.y = 0.404;
        home_pose.position.z = 0.620;  // Final home z position

        home_pose.orientation.x = 0.664;
        home_pose.orientation.y = 0.23933;
        home_pose.orientation.z = 0.669;
        home_pose.orientation.w = -0.2327;

        moveToPose(home_pose);
    }

    // Move to hole position (x, y, fixed z)
    void moveToHole(double x, double y, double z)
    {
        RCLCPP_INFO(this->get_logger(), "Moving to hole position at x: %f, y: %f,z: %f", x, y, z);

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;  // Fixed z for hole

        target_pose.orientation.x = 0.664;
        target_pose.orientation.y = 0.23933;
        target_pose.orientation.z = 0.669;
        target_pose.orientation.w = -0.2327; 
        moveToPose(target_pose);
    }

    // Move to tool position (shift x by 0.7, then go straight down 0.25)
    void moveToTool()
    {
        RCLCPP_INFO(this->get_logger(), "Moving to tool position.");

        geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose().pose;
        
        

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_pose.position.x + 0.620; 
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.3;  // Safe z height
        target_pose.orientation.x = 0.664;
        target_pose.orientation.y = 0.23933;
        target_pose.orientation.z = 0.669;
        target_pose.orientation.w = -0.2327; 
        moveToPose(target_pose);

        // Now go straight down
        // target_pose.position.z = 0.15;  // Move down by 0.25
        // moveToPose(target_pose);
    }

    // Helper function to move to a given pose
    void moveToPose(const geometry_msgs::msg::Pose &pose)
    {
        move_group_interface_->setPoseTarget(pose);
        move_group_interface_->setPathConstraints(joint_constraints_);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);


        if (success) {
            RCLCPP_INFO(this->get_logger(), "Executing move.");
            move_group_interface_->execute(plan);
            publishArmStatus("done");
        } else {
            RCLCPP_WARN(this->get_logger(), "Movement planning failed.");
            publishArmStatus("fail");
        }

        move_group_interface_->clearPathConstraints();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmMovement>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
