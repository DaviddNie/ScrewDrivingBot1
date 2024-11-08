#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/vision_cmd.hpp"
#include "interfaces/srv/brain_cmd.hpp"
#include "interfaces/srv/arm_cmd.hpp"
#include "interfaces/srv/brain_routine_cmd.hpp"
#include "interfaces/srv/end_effector_cmd.hpp"
#include "interfaces/srv/real_coor_cmd.hpp"
#include "interfaces/srv/publish_ooi_cmd.hpp"
#include <string>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <queue>
#include <condition_variable>
#include <mutex>

class Brain : public rclcpp::Node
{
public:
	Brain() : Node("brain"), is_busy(false), movement_finished(true)
	{
        // Create Mutually Exclusive Callback Groups
        vision_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        movement_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        end_effector_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        brain_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        ooi_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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

		armClient_ = create_client<interfaces::srv::ArmCmd>(
			"arm_srv", rmw_qos_profile_services_default, movement_cb_group_);

		ooiServerClient_ = create_client<interfaces::srv::PublishOoiCmd>(
			"ooi_srv", rmw_qos_profile_services_default, ooi_cb_group_);

		tfServerClient_ = create_client<interfaces::srv::RealCoorCmd>(
			"tf_srv", rmw_qos_profile_services_default, ooi_cb_group_);

		publishBrainStatus("Brain Node initiated");
	}

private:
	void processBrainRoutineService(const std::shared_ptr<interfaces::srv::BrainRoutineCmd::Request> request,
				 std::shared_ptr<interfaces::srv::BrainRoutineCmd::Response> response) {
		std::string command = request->command;

		if (is_busy) {
			response->output = busy;
			publishBrainStatus("Brain is busy");
			return;
		}

		is_busy = true;
		if (command == screwdrivingRoutine) {
			publishBrainStatus("Initiating Screwdriving Routine");
			response->output = runScrewdrivingRoutine();	
		} else if (command == newScrewdrivingRoutine) {
			publishBrainStatus("Initiating New Screwdriving Routine");
			response->output = runNewScrewdrivingRoutine();
		} else {
			// Unknown command
			response->output = unknown;
			publishBrainStatus("Routine Service received unrecognized command: " + command);
		}
		is_busy = false;	
	}

	std_msgs::msg::Int32 runScrewdrivingRoutine() {
		publishBrainStatus("Waiting for movement to finish...");
		
		// (Movement) Go to Birds-eye pose
		int birdseye_movement_success = callMovementModule(home, geometry_msgs::msg::Point());
		

        std::unique_lock<std::mutex> lock(movement_mutex_);
        movement_cv_.wait(lock, [this] { return movement_finished; });
		movement_mutex_.unlock();

		if (!birdseye_movement_success) {
			publishBrainStatus("ERROR: Birdseye movement failed, terminating...");
			return failure; 
		}
		publishBrainStatus("Movement finished! Continuing...");

		// Get screw centriods in image frame
		geometry_msgs::msg::PoseArray output = callVisionModule(birdsEyeCmd);

		// Create a queue to store centroids
		// Note that the pose are global in terms of the image coordinates, not with respect to base_link
		std::queue<geometry_msgs::msg::Pose> centroidQueue;
		for (const auto& pose : output.poses) {
			centroidQueue.push(pose);
		}

		if (centroidQueue.empty()) {
			publishBrainStatus("ERROR: Queue is empty!!!");
		}

		// Process each centroid in the queue
		while (!centroidQueue.empty()) {
			geometry_msgs::msg::Pose currentCentroid = centroidQueue.front();
			centroidQueue.pop();

			double x = currentCentroid.position.x;
        	double y = currentCentroid.position.y;

			publishBrainStatus("Processing (" + std::to_string(x) + "," + std::to_string(y) + ")");

			// (Transformation) Set as "OOI" frame, convert to RealCoor (with respect to base_link)
			bool status = callOOIModule(currentCentroid);

			if (!status) {
				publishBrainStatus("ERROR: OOI Module failed to establish `OOI` Frame");
				return failure; 
			}

			// Get relative position of "OOI" with resepct to "base_link"
			geometry_msgs::msg::Pose realPose = getRealPoseFromTF();

			if (checkInvalidPose(realPose)) {
				publishBrainStatus("ERROR: TF Transform from OOI to base_link is invalid");
				return failure; 
			}

			// Manually set z to 0.025
			realPose.position.z = 0.025;
			realPose.position.x = realPose.position.x -0.061284;
			realPose.position.y = realPose.position.y -0.049078;

			publishBrainStatus("Real Coor: x= " + std::to_string(realPose.position.x) + ", y=" + std::to_string(realPose.position.y) + 
				", z=" + std::to_string(realPose.position.z));
			
			// (Movement) Move to (x y 0.025)
			status = callMovementModule(hole, realPose.position);

	        movement_cv_.wait(lock, [this] { return movement_finished; });
			movement_mutex_.unlock();

			if (!status) {
				publishBrainStatus("ERROR: Move to z failed");
				return failure; 
			}

			// (Movement) Back to home
			geometry_msgs::msg::Point point;
			status = callMovementModule(home, point);

	        movement_cv_.wait(lock, [this] { return movement_finished; });
			movement_mutex_.unlock();

			if (!status) {
				publishBrainStatus("ERROR: Move to 0.3 in z failed");
				return failure; 
			}
	
			// Screwdriving
			// callEndEffectorModule(turnLightOn);
			// callEndEffectorModule(startScrewDiving);
			// callEndEffectorModule(turnLightOff);

		}

		publishBrainStatus("Screwdriving Routine Complete");
		return success;
	}

	std_msgs::msg::Int32 runNewScrewdrivingRoutine() {
		publishBrainStatus("Waiting for movement to finish...");
		callEndEffectorModule(turnLightOn);
		
		// ---- (Movement) Go to Birds-eye pose ----
		int birdseye_movement_success = callMovementModule(home, geometry_msgs::msg::Point());
		
        std::unique_lock<std::mutex> lock(movement_mutex_);
        movement_cv_.wait(lock, [this] { return movement_finished; });
		movement_mutex_.unlock();

		if (!birdseye_movement_success) {
			publishBrainStatus("ERROR: Birdseye movement failed, terminating...");
			return failure; 
		}
		publishBrainStatus("Movement finished! Continuing...");
		

		// ---- Get screw centriods in image frame ---- 
		geometry_msgs::msg::PoseArray output = callVisionModule(birdsEyeCmd);

		// ---- Create a queue to store centroids ----
			// Note that the pose are global in terms of the image coordinates, not with respect to base_link
		std::queue<geometry_msgs::msg::Pose> centroidQueue;
		for (const auto& pose : output.poses) {
			centroidQueue.push(pose);
		}

		if (centroidQueue.empty()) {
			publishBrainStatus("ERROR: Queue is empty!!!");
		}

		// ---- Process each centroid in the queue ----
		while (!centroidQueue.empty()) {
			geometry_msgs::msg::Pose currentCentroid = centroidQueue.front();
			centroidQueue.pop();

			double x = currentCentroid.position.x;
        	double y = currentCentroid.position.y;

			publishBrainStatus("Processing (" + std::to_string(x) + "," + std::to_string(y) + ")");

			// ---- (Transformation) Set as "OOI" frame, convert to RealCoor (with respect to base_link) ----
			bool status = callOOIModule(currentCentroid);

			if (!status) {
				publishBrainStatus("ERROR: OOI Module failed to establish `OOI` Frame");
				return failure; 
			}

			// Get relative position of "OOI" with resepct to "base_link"
			geometry_msgs::msg::Pose realPose = getRealPoseFromTF();

			if (checkInvalidPose(realPose)) {
				publishBrainStatus("ERROR: TF Transform from OOI to base_link is invalid");
				return failure; 
			}

			// (Movement) Move to (x y z)
			realPose.position.z = 0.025; 	
			realPose.position.x = realPose.position.x -0.061284;
			realPose.position.y = realPose.position.y -0.049078;

			publishBrainStatus("Real Coor: x= " + std::to_string(realPose.position.x) + ", y=" + std::to_string(realPose.position.y) + 
				", z=" + std::to_string(realPose.position.z));
	
			status = callMovementModule(hole, realPose.position);

	        movement_cv_.wait(lock, [this] { return movement_finished; });
			movement_mutex_.unlock();

			if (!status) {
				publishBrainStatus("ERROR: Move to z failed");
				return failure; 
			}

			// (Movement) Move down for screwdriving action
				// Set the position
			geometry_msgs::msg::Pose downPose;
			downPose.position.x = 0.0;  
			downPose.position.y = 0.0; 
			downPose.position.z = -0.01;

			callEndEffectorModule(startScrewDiving);
			status = callMovementModule(tool, downPose.position);
			
			movement_cv_.wait(lock, [this] { return movement_finished; });
			movement_mutex_.unlock();

			if (!status){
				publishBrainStatus("ERROR: screwing failed");
				callEndEffectorModule(turnLightOff);
				return failure; 
			}
			callEndEffectorModule(turnLightOff);

			// (Movement) Move up to return after screwdriving action
				// Set the position
			geometry_msgs::msg::Pose upPose;
			upPose.position.x = 0.0;
			upPose.position.y = 0.0;
			upPose.position.z = 0.01;


			status=callMovementModule(tool, upPose.position);

			movement_cv_.wait(lock, [this] { return movement_finished; });
			movement_mutex_.unlock();

			if (!status){
				publishBrainStatus("ERROR: go up a little failed");
				return failure; 
			}

			// (Movement) Back to home
			geometry_msgs::msg::Point point;
			status = callMovementModule(home, point);
			callEndEffectorModule(turnLightOff);

	        movement_cv_.wait(lock, [this] { return movement_finished; });
			movement_mutex_.unlock();

			if (!status) {
				publishBrainStatus("ERROR: Move in z failed");
				return failure; 
			}
		}
		publishBrainStatus("New Screwdriving Routine Complete");
		return success;
	}

	geometry_msgs::msg::Pose getRealPoseFromTF() {
		auto request = std::make_shared<interfaces::srv::RealCoorCmd::Request>();
		request->command = default_;

		auto future_result = tfServerClient_->async_send_request(request);
        
		geometry_msgs::msg::Pose result = future_result.get()->real_pose;
        
		return result;
	}

	bool checkInvalidPose(geometry_msgs::msg::Pose pose) {
		return pose.position.x == 0.0 &&
           pose.position.y == 0.0 &&
           pose.position.z == 0.0;
	}
	
	void processBrainService(const std::shared_ptr<interfaces::srv::BrainCmd::Request> request,
				 std::shared_ptr<interfaces::srv::BrainCmd::Response> response) {
		std::string module = request->module;
		std::string command = request->command;
		geometry_msgs::msg::Pose pose = request->pose;

		if (module == visionModule) {
			publishBrainStatus("Calling Vision Module");
			geometry_msgs::msg::PoseArray output = callVisionModule(command);

			response->output = success;
		} else if (module == movementModule) {
			publishBrainStatus("Calling Movement Module");
			// callMovementModule(command);

			response->output = success;
		} else if (module == endEffectorModule) {
			publishBrainStatus("Calling End Effector Module");
			callEndEffectorModule(command);

			response->output = success;
		} else if (module == ooiModule) {
			publishBrainStatus("Calling ooi Module");
			bool output = callOOIModule(pose);

			if (output) {
				response->output = success;
			} else {
				response->output = failure;
			}
		}else {
			publishBrainStatus("Error!!! Unknown Module name");
			
			response->output = failure;
		}
	}

	void publishBrainStatus(std::string msg) {
		std_msgs::msg::String tmpHolder;
		tmpHolder.data = msg;
		brainStatusPublisher->publish(tmpHolder);
	}

	int callMovementModule(const std::string &mode, const geometry_msgs::msg::Point point) {
		setMovementProcessing();

		auto request = std::make_shared<interfaces::srv::ArmCmd::Request>();
		request->mode = mode;  // Set the command (e.g., "START SCREWDRIVING" or "GET_STATUS" or "TURN_LIGHT_ON" or "TURN_LIGHT_OFF")
		request->point = point;
		
		// Call the service
		auto result_future = armClient_->async_send_request(request);
		auto response = result_future.get();

		setMovementFinished();

		return response->success;
	}

	// pause execution until movement is finished  
	void setMovementProcessing() {
		std::lock_guard<std::mutex> lock(movement_mutex_);
		movement_finished = false;
	}

	void setMovementFinished() {
		std::lock_guard<std::mutex> lock(movement_mutex_);
		movement_finished = true;
		movement_cv_.notify_all();
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
        
		return result;
	}

	
	bool callOOIModule(const geometry_msgs::msg::Pose input) {
		auto request = std::make_shared<interfaces::srv::PublishOoiCmd::Request>();
		request->img_pose = input;

		auto future_result = ooiServerClient_->async_send_request(request);
        
		bool success = future_result.get()->success;
        
		return success;
	}
	
	// Callback groups
    rclcpp::CallbackGroup::SharedPtr movement_cb_group_;
    rclcpp::CallbackGroup::SharedPtr vision_cb_group_;
    rclcpp::CallbackGroup::SharedPtr end_effector_cb_group_;
    rclcpp::CallbackGroup::SharedPtr brain_cb_group_;
    rclcpp::CallbackGroup::SharedPtr ooi_cb_group_;

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr brainStatusPublisher;

	rclcpp::Service<interfaces::srv::BrainCmd>::SharedPtr brainService;
	rclcpp::Service<interfaces::srv::BrainRoutineCmd>::SharedPtr brainRoutineService;

	rclcpp::Client<interfaces::srv::VisionCmd>::SharedPtr visionClient_;

	rclcpp::Client<interfaces::srv::EndEffectorCmd>::SharedPtr endEffectorClient_;

	rclcpp::Client<interfaces::srv::ArmCmd>::SharedPtr armClient_;


	rclcpp::Client<interfaces::srv::PublishOoiCmd>::SharedPtr ooiServerClient_;
	rclcpp::Client<interfaces::srv::RealCoorCmd>::SharedPtr tfServerClient_;

	// Module constants
	std::string const visionModule = "vision";
	std::string const movementModule = "movement";
	std::string const endEffectorModule = "endEffector";
	std::string const ooiModule = "ooi";

	// Routine commands
	std::string const screwdrivingRoutine = "screwdriving";
	std::string const newScrewdrivingRoutine = "screwdriving2";

	// vision commands
	std::string const birdsEyeCmd = "birds_eye";
	std::string const clibrateCmd = "calibrate";

	// end-effector commands
	std::string const startScrewDiving = "START SCREWDRIVING";
	std::string const getStatus = "GET_STATUS";
	std::string const turnLightOn = "TURN_LIGHT_ON";
	std::string const turnLightOff = "TURN_LIGHT_OFF";

	// arm modes
	std::string const home = "home";
	std::string const hole = "hole";
	std::string const tool = "tool";

	// tf server commands
	std::string const default_ = "default";

	bool is_busy;
	bool movement_finished;
    std::mutex movement_mutex_;
    std::condition_variable movement_cv_;


	std_msgs::msg::Int32 const success = std_msgs::msg::Int32().set__data(1);
	std_msgs::msg::Int32 const failure = std_msgs::msg::Int32().set__data(0);
	std_msgs::msg::Int32 const unknown = std_msgs::msg::Int32().set__data(2);
	std_msgs::msg::Int32 const busy = std_msgs::msg::Int32().set__data(3);
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