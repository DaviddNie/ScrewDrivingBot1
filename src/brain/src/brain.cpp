#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/vision_cmd.hpp"
#include <string>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cmath>
#include <tf2_ros/transform_listener.h>

class Brain : public rclcpp::Node
{
public:
	Brain() : Node("brain")
	{
        // Create Mutually Exclusive Callback Groups
        vision_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        movement_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        end_effector_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		visionStatusSub_ = this->create_subscription<std_msgs::msg::String>(
			"vision_status", 10,
			std::bind(&Brain::visionStatusCallback, this, std::placeholders::_1));

	    visionClient_ = create_client<interfaces::srv::VisionCmd>
			("vision_srv", rmw_qos_profile_services_default, vision_cb_group_);		
	}

private:
	geometry_msgs::msg::PoseArray callVisionService(const std::string &command) {
		auto request = std::make_shared<interfaces::srv::VisionCmd::Request>();
		request->command = command;

		auto future_result = visionClient_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS) {

            return future_result.get()->pose_array;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call vision service");
            return geometry_msgs::msg::PoseArray();
        }
	}

	void visionStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
		RCLCPP_INFO(this->get_logger(), "Vision Status: %s", msg->data.c_str());
	}
	
	// Callback groups
    rclcpp::CallbackGroup::SharedPtr movement_cb_group_;
    rclcpp::CallbackGroup::SharedPtr vision_cb_group_;
    rclcpp::CallbackGroup::SharedPtr end_effector_cb_group_;

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr visionStatusSub_;	

	rclcpp::Client<interfaces::srv::VisionCmd>::SharedPtr visionClient_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto server = std::make_shared<Brain>();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(server);
	executor.spin();
	
	return 0;
}