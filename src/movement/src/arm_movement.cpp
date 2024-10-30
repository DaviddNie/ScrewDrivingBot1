#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/quaternion.hpp>

// Define orientation as a global variable for the class
const geometry_msgs::msg::Quaternion DEFAULT_ORIENTATION = []{
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = 0.643;
    orientation.y = 0.288;
    orientation.z = 0.659;
    orientation.w = -0.263;
    return orientation;
}();

auto generateCollisionObject(float sx, float sy, float sz, float x, float y, float z, std::string frame_id, std::string id) {
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

        // publisher for movement status
        movement_status_pub_ = this->create_publisher<std_msgs::msg::String>("arm_status", 10);

        // Initialize MoveIt move group interface
        move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");

        move_group_interface_->setPlanningTime(10.0);
        move_group_interface_->setNumPlanningAttempts(10);
        move_group_interface_->setGoalTolerance(0.01);  // 1 cm tolerance
        std::string frame_id = move_group_interface_->getPlanningFrame();

        auto col_object_backWall = generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall");
        auto col_object_sideWall = generateCollisionObject(0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall");
        auto col_object_table = generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 0.05, frame_id, "table");
        auto col_object_ceiling = generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 1.2, frame_id, "ceiling");


        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_interface.applyCollisionObject(col_object_backWall);
        planning_scene_interface.applyCollisionObject(col_object_sideWall);
        planning_scene_interface.applyCollisionObject(col_object_table);
        planning_scene_interface.applyCollisionObject(col_object_ceiling);

        moveit_msgs::msg::JointConstraint shoulder_pan;
        shoulder_pan.joint_name = "shoulder_pan_joint";
        shoulder_pan.position = -M_PI/  2;
        shoulder_pan.tolerance_above = M_PI / 2;
        shoulder_pan.tolerance_below = M_PI / 2;
        shoulder_pan.weight = 1.0;

        moveit_msgs::msg::JointConstraint shoulder_lift;
        shoulder_lift.joint_name = "shoulder_lift_joint";
        shoulder_lift.position = M_PI/2;
        shoulder_lift.tolerance_above = M_PI / 2;
        shoulder_lift.tolerance_below = M_PI / 2;
        shoulder_lift.weight = 1.0;

        moveit_msgs::msg::JointConstraint elbow;
        elbow.joint_name = "elbow_joint";
        elbow.position = M_PI/2;
        elbow.tolerance_above = M_PI / 2;
        elbow.tolerance_below = M_PI / 2;
        elbow.weight = 1.0;

        moveit_msgs::msg::JointConstraint wrist1;
        wrist1.joint_name = "wrist_1_joint";
        wrist1.position = M_PI;
        wrist1.tolerance_above = M_PI / 2;
        wrist1.tolerance_below = M_PI / 2;
        wrist1.weight = 1.0;

        moveit_msgs::msg::JointConstraint wrist2;
        wrist2.joint_name = "wrist_2_joint";
        wrist2.position = M_PI/2;
        wrist2.tolerance_above = M_PI / 2;
        wrist2.tolerance_below = M_PI / 2;
        wrist2.weight = 1.0;

        moveit_msgs::msg::JointConstraint wrist3;
        wrist3.joint_name = "wrist_3_joint";
        wrist3.position = M_PI/2;
        wrist3.tolerance_above = M_PI / 2;
        wrist3.tolerance_below = M_PI / 2;
        wrist3.weight = 1.0;

        // Add all constraints to the joint_constraints_ list
        // joint_constraints_.joint_constraints.push_back(shoulder_pan);
        // joint_constraints_.joint_constraints.push_back(shoulder_lift);
        // joint_constraints_.joint_constraints.push_back(elbow);
        //joint_constraints_.joint_constraints.push_back(wrist1);
        // joint_constraints_.joint_constraints.push_back(wrist2);
        // joint_constraints_.joint_constraints.push_back(wrist3);


        RCLCPP_INFO(this->get_logger(), "ArmMovement node initialized.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr movement_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr movement_done_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr movement_status_pub_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    moveit_msgs::msg::Constraints joint_constraints_;

    // Callback for movement commands
    void movementCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received movement command: %s", msg->data.c_str());
        std::stringstream ss;
        ss << "Received command: " << msg->data;
        publishArmStatus(ss.str());

        if (msg->data == "home") {
            moveToHome();
        } else if (msg->data.rfind("hole", 0) == 0) {
            // Parse hole coordinates from message (e.g., "hole,x,y,z")
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
    }

    // Move to home position
    void moveToHome()
    {
        RCLCPP_INFO(this->get_logger(), "Moving to home position.");
        publishArmStatus("moving to home");

        geometry_msgs::msg::Pose home_pose;
        home_pose.position.x = 0.351;
        home_pose.position.y = 0.328;
        home_pose.position.z = 0.50;  // Final home z position

        home_pose.orientation = DEFAULT_ORIENTATION;  // Use the global orientation

        moveToPose(home_pose);
    }

    // Move to hole position (x, y, z)
    void moveToHole(double x, double y, double z)
    {
        RCLCPP_INFO(this->get_logger(), "Moving to hole position at x: %f, y: %f, z: %f", x, y, z);
        std::stringstream ss;
        ss << "Moving to hole position at x: " << x << ", y: " << y << ", z: " << z;
        publishArmStatus(ss.str());
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation = DEFAULT_ORIENTATION;

        

        moveToPose(target_pose);
    }

    // Move to tool position
    void moveToTool()
    {
        RCLCPP_INFO(this->get_logger(), "Moving to tool position.");
        publishArmStatus("moving to tool");

        geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose().pose;
        
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_pose.position.x + 0.620; 
        target_pose.position.y = current_pose.position.y;
        target_pose.position.z = 0.3;  // Safe z height
        target_pose.orientation = DEFAULT_ORIENTATION;

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
            publishArmStatus("executing now");
            move_group_interface_->execute(plan);
            move_group_interface_->stop();
            publishArmStatus("movement done");
            publishArmDone("done");
        } else {
            RCLCPP_WARN(this->get_logger(), "Movement planning failed.");
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

    void publishArmStatus(const std::string &status)
    {
        std_msgs::msg::String status_msg;
        status_msg.data = status;
        movement_status_pub_->publish(status_msg);
        RCLCPP_INFO(this->get_logger(), "Published arm status: %s", status.c_str());
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
