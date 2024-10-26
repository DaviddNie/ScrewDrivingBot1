#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/vision_cmd.hpp"
#include "interfaces/srv/brain_cmd.hpp"
#include "interfaces/srv/brain_routine_cmd.hpp"
#include "interfaces/srv/end_effector_cmd.hpp"
#include <string>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <queue>

class Brain : public rclcpp::Node
{
public:
	Brain() : Node("brain")
	{
        // Create Mutually Exclusive Callback Groups
        vision_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        movement_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        end_effector_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        brain_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		brainStatusPublisher = this->create_publisher<std_msgs::msg::String>
			("brain_status", 10);

		brainService = create_service<interfaces::srv::BrainCmd>(
			"brain_srv",
			std::bind(&Brain::processBrainService, this, std::placeholders::_1, std::placeholders::_2),
			rmw_qos_profile_services_default, brain_cb_group_);

		brainRoutineService = create_service<interfaces::srv::BrainRoutineCmd>(
			"brain_routine_srv",
			std::bind(&Brain::processBrainRoutineService, this, std::placeholders::_1, std::placeholders::_2),
			rmw_qos_profile_services_default, brain_cb_group_);

	    visionClient_ = create_client<interfaces::srv::VisionCmd>
			("vision_srv", rmw_qos_profile_services_default, vision_cb_group_);		

		endEffectorClient_ = create_client<interfaces::srv::EndEffectorCmd>(
			"end_effector_srv", rmw_qos_profile_services_default, end_effector_cb_group_);

		publishBrainStatus("Brain Node initiated");
	}

private:
	void processBrainRoutineService(const std::shared_ptr<interfaces::srv::BrainRoutineCmd::Request> request,
				 std::shared_ptr<interfaces::srv::BrainRoutineCmd::Response> response) {
		std::string command = request->command;

		if (command == screwdrivingRoutine) {
			publishBrainStatus("Initiating Screwdriving Routine");

			response->output = runScrewdrivingRoutine();		
		} else {
			// Unknown command
			response->output = unknown;
			publishBrainStatus("Routine Service received unrecognized command: " + command);
		}
	}

	std_msgs::msg::Int32 runScrewdrivingRoutine() {
		// TODO: (Movement) Go to Birds-eye pose
			
		// Get screw centriods
		geometry_msgs::msg::PoseArray output = callVisionModule(birdsEyeCmd);

		// Create a queue to store centroids
		std::queue<geometry_msgs::msg::Pose> centroidQueue;
		for (const auto& pose : output.poses) {
			centroidQueue.push(pose);
		}

		// Process each centroid in the queue
		while (!centroidQueue.empty()) {
			geometry_msgs::msg::Pose currentCentroid = centroidQueue.front();
			centroidQueue.pop();

			double x = currentCentroid.position.x;
        	double y = currentCentroid.position.y;

			publishBrainStatus("Processing (" + std::to_string(x) + "," + std::to_string(y) + ")");

			// TODO: (Movement) Move to 0.3 in z-axis

			// TODO: (Movement) Move to (x y 0.3)

			// TODO: (Vision) Re-tune

			// TODO: (Movement) Move to (new_x, new_y, 0.3)

			// TODO: Check if displacement is more than 0.05, Re-tune if so

			// TODO: (Movement) Go-down (assume known height)

			// Screwdriving
			// callEndEffectorModule(startScrewDiving);

		}

		publishBrainStatus("Screwdriving Routine Complete");
		return success;
	}

	void processBrainService(const std::shared_ptr<interfaces::srv::BrainCmd::Request> request,
				 std::shared_ptr<interfaces::srv::BrainCmd::Response> response) {
		std::string module = request->module;
		std::string command = request->command;

		if (module == visionModule) {
			publishBrainStatus("Calling Vision Module");
			geometry_msgs::msg::PoseArray output = callVisionModule(command);

			response->output = success;
		} else if (module == movementModule) {
			publishBrainStatus("Calling Movement Module");
			callMovementModule(command);

			response->output = success;
		} else if (module == endEffectorModule) {
			publishBrainStatus("Calling End Effector Module");
			callEndEffectorModule(command);

			response->output = success;
		} else {
			publishBrainStatus("Error!!! Unknown Module name");
			
			response->output = failure;
		}
	}

	void publishBrainStatus(std::string msg) {
		std_msgs::msg::String tmpHolder;
		tmpHolder.data = msg;
		brainStatusPublisher->publish(tmpHolder);
	}

	int callMovementModule(const std::string &command) {
		std::string bla = command;
		return 0;
	}

	int callEndEffectorModule(const std::string &command) {
		auto request = std::make_shared<interfaces::srv::EndEffectorCmd::Request>();
		request->command = command;  // Set the command (e.g., "START SCREWDRIVING" or "GET_STATUS" or "TURN_LIGHT_ON" or "TURN_LIGHT_OFF")

		// Wait for the service to be available
		if (!endEffectorClient_->wait_for_service(std::chrono::seconds(1))) {
			publishBrainStatus("End Effector service not available.");
			return 0;
		}

		// Call the service
		auto result_future = endEffectorClient_->async_send_request(request);
		auto status = result_future.wait_for(std::chrono::seconds(2));

		if (status == std::future_status::ready) {
			auto response = result_future.get();
			publishBrainStatus("End Effector Response: " + response->message);
			return response->success;
		} else {
			publishBrainStatus("Failed to call End Effector service.");
			return 0;
		}
	}
	
	geometry_msgs::msg::PoseArray callVisionModule(const std::string &command) {
		auto request = std::make_shared<interfaces::srv::VisionCmd::Request>();
		request->command = command;


		auto future_result = visionClient_->async_send_request(request);
        
		geometry_msgs::msg::PoseArray result = future_result.get()->pose_array;
		
		publishBrainStatus("Result back to Brain");
        
		return result;
	}
	
	// Callback groups
    rclcpp::CallbackGroup::SharedPtr movement_cb_group_;
    rclcpp::CallbackGroup::SharedPtr vision_cb_group_;
    rclcpp::CallbackGroup::SharedPtr end_effector_cb_group_;
    rclcpp::CallbackGroup::SharedPtr brain_cb_group_;

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr brainStatusPublisher;

	rclcpp::Service<interfaces::srv::BrainCmd>::SharedPtr brainService;
	rclcpp::Service<interfaces::srv::BrainRoutineCmd>::SharedPtr brainRoutineService;

	rclcpp::Client<interfaces::srv::VisionCmd>::SharedPtr visionClient_;

	rclcpp::Client<interfaces::srv::EndEffectorCmd>::SharedPtr endEffectorClient_;

	// Module constants
	std::string const visionModule = "vision";
	std::string const movementModule = "movement";
	std::string const endEffectorModule = "endEffector";

	// Routine commands
	std::string const screwdrivingRoutine = "screwdriving";

	// vision commands
	std::string const birdsEyeCmd = "birds_eye";
	std::string const clibrateCmd = "calibrate";

	// end-effector commands
	std::string const startScrewDiving = "START SCREWDRIVING";
	std::string const getStatus = "GET_STATUS";
	std::string const turnLightOn = "TURN_LIGHT_ON";
	std::string const turnLightOff = "TURN_LIGHT_OFF";

	std_msgs::msg::Int32 const success = std_msgs::msg::Int32().set__data(1);
	std_msgs::msg::Int32 const failure = std_msgs::msg::Int32().set__data(0);
	std_msgs::msg::Int32 const unknown = std_msgs::msg::Int32().set__data(2);
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