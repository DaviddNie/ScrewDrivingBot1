#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "interfaces/srv/arm_cmd.hpp"   // Correct your service package
#include "std_msgs/msg/string.hpp"

class ArmBrain : public rclcpp::Node
{
public:
    ArmBrain() : Node("arm_brain")
    {
        // Create mutually exclusive callback groups
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        movement_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create service with callback group
        service_ = this->create_service<interfaces::srv::ArmCmd>("arm_srv",
            std::bind(&ArmBrain::commandCallback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            service_callback_group_);

        // Create publisher to communicate with arm_movement
        movement_pub_ = this->create_publisher<std_msgs::msg::String>("move_to", 10);

        // Create subscriber to check if movement is done, using the movement callback group
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = movement_callback_group_;
        movement_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "done_move", 10, std::bind(&ArmBrain::movementStatusCallback, this, std::placeholders::_1), sub_options);

        RCLCPP_INFO(this->get_logger(), "ArmBrain node initialized with mutually exclusive callback groups.");
    }

private:
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::CallbackGroup::SharedPtr movement_callback_group_;
    rclcpp::Service<interfaces::srv::ArmCmd>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr movement_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr movement_status_sub_;

    bool movement_done_ = true;  // Tracks if movement has finished
    bool movement_success;

    // Service callback to handle commands
    void commandCallback(const std::shared_ptr<interfaces::srv::ArmCmd::Request> request,
                         std::shared_ptr<interfaces::srv::ArmCmd::Response> response)
    {
        if (!movement_done_) {
            RCLCPP_WARN(this->get_logger(), "Arm is busy, please wait for the current task to finish or movement failed.");
            response->success = false;
            return;
        }
        
        movement_done_ = false;  // Block new commands while movement is ongoing
        movement_success=false;

        // Publish movement command based on the request mode
        std_msgs::msg::String move_msg;

        if (request->mode == "home") {
            move_msg.data = "home";
        } else if (request->mode == "hole") {
            move_msg.data = "hole," + std::to_string(request->point.x) + "," +
                            std::to_string(request->point.y) + "," +
                            std::to_string(request->point.z);
        } else if (request->mode == "tool") {
            move_msg.data = "tool";
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown mode: %s", request->mode.c_str());
            response->success = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Publishing movement command: %s", move_msg.data.c_str());
        movement_pub_->publish(move_msg);

        while (!movement_done_ && rclcpp::ok()) {
            rclcpp::sleep_for(std::chrono::milliseconds(100));  // Wait 100ms between checks
        }
        response->success = movement_success;
        
        
    }

    // Callback to handle when movement is done
    void movementStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "done") {
            RCLCPP_INFO(this->get_logger(), "Movement complete.");
            movement_success = true;  // Allow new commands
        } else {
            RCLCPP_INFO(this->get_logger(), "Movement failed.");
            movement_success = false; 
        }
        movement_done_ = true; 
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Use a multi-threaded executor to process callbacks concurrently
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ArmBrain>();

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
