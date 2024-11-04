#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include "interfaces/srv/real_coor_cmd.hpp"
#include "interfaces/srv/publish_ooi_cmd.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class OOIServer : public rclcpp::Node
{
	public:
		OOIServer() : Node("camera_dy_tf_broadcaster")
		{
			static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

			initTrans();

			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

			timer_ = this->create_wall_timer(
					std::chrono::milliseconds(10),
					std::bind(&OOIServer::broadcastTransform, this));

			ooiService = create_service<interfaces::srv::PublishOoiCmd>(
				"ooi_srv",
				std::bind(&OOIServer::processOOIService, this, std::placeholders::_1, std::placeholders::_2));

			tfService = create_service<interfaces::srv::RealCoorCmd>(
				"tf_srv",
				std::bind(&OOIServer::processTFService, this, std::placeholders::_1, std::placeholders::_2));

			ooiServerStatusPublisher = this->create_publisher<std_msgs::msg::String>
				("ooi_server_status", 10);
		}

	private:
		void initTrans() {
			transform_stamped.header.stamp = now();
			transform_stamped.header.frame_id = "camera_link";
			transform_stamped.child_frame_id = "OOI";

			// Identity quaternion (no rotation)
			transform_stamped.transform.rotation.x = 0.0;
			transform_stamped.transform.rotation.y = 0.0;
			transform_stamped.transform.rotation.z = 0.0;
			transform_stamped.transform.rotation.w = 1.0;
		}

		void processOOIService(const std::shared_ptr<interfaces::srv::PublishOoiCmd::Request> request,
				 std::shared_ptr<interfaces::srv::PublishOoiCmd::Response> response) {
			publishServerStatus("OOI Server: Request received");
			
			geometry_msgs::msg::Pose inputPose = request->img_pose;

			double x = inputPose.position.x;
			double y = inputPose.position.y;
			double z = inputPose.position.z;

			processAndSendTransform(x, y, z);

			response->success = true;

			publishServerStatus("OOI Server: Processing complete!");
		}

		void processTFService(const std::shared_ptr<interfaces::srv::RealCoorCmd::Request> request,
				 std::shared_ptr<interfaces::srv::RealCoorCmd::Response> response) {
			publishServerStatus("TF Server: Request received");
			
			try
			{
				// Attempt to look up the transform from "base_link" to "OOI"
				geometry_msgs::msg::TransformStamped transformStamped = 
					tf_buffer_->lookupTransform("base_link", "OOI", rclcpp::Time(0));
				
				double x = transformStamped.transform.translation.x;
				double y = transformStamped.transform.translation.y;
				double z = transformStamped.transform.translation.z;

				response->real_pose.position.x = x;
				response->real_pose.position.y = y;
				response->real_pose.position.z = z;

				publishServerStatus("Transform: x= " + std::to_string(x) + ", y=" + std::to_string(y) + ", z=" + std::to_string(z));
				publishServerStatus("TF Server: Processing complete!");
			}
			catch (const tf2::TransformException & ex)
			{
				publishServerStatus("ERROR: Transform failed - " + std::string(ex.what()));

				// Default -> invalid
				response->real_pose = geometry_msgs::msg::Pose();			
			}

		}

		void processAndSendTransform(double x, double y, double z) {
			// This is messed up, we succeed by trial and error
			// the weird order of "xyz" come from the orientation of the camera with respect to OOI
			// THe "-" signs come from the fact that the coordinate frame set by opencv is different from reality (the origin is at opposite corner)
			// 
			// good thing is we can just apply this later on
			//
			// Assign position to transformation
			transform_stamped.transform.translation.x = z;
			transform_stamped.transform.translation.y = -y;
			transform_stamped.transform.translation.z = -x;

			static_tf_broadcaster_->sendTransform(transform_stamped);
		}

		void publishServerStatus(std::string msg) {
			std_msgs::msg::String tmpHolder;
			tmpHolder.data = msg;
			ooiServerStatusPublisher->publish(tmpHolder);
		}

		void broadcastTransform() {
			transform_stamped.header.stamp = this->now();
			static_tf_broadcaster_->sendTransform(transform_stamped);

			// publishServerStatus("updated transform_stamped.child_frame_id is " + transform_stamped.child_frame_id);
		}

		std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

		rclcpp::Service<interfaces::srv::PublishOoiCmd>::SharedPtr ooiService;
		rclcpp::Service<interfaces::srv::RealCoorCmd>::SharedPtr tfService;
		geometry_msgs::msg::TransformStamped transform_stamped;

		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ooiServerStatusPublisher;

		rclcpp::TimerBase::SharedPtr timer_;

		std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OOIServer>());
	rclcpp::shutdown();
	return 0;
}

// [INFO] [1730330669.029399428] [vision_test]: Vision Status - Point 1 is at pixel x = 147.41, y = 69.91
// [INFO] [1730330669.030878990] [vision_test]: Vision Status - Point 2 is at pixel x = 173.82, y = 19.11
// [INFO] [1730330669.032910818] [vision_test]: Vision Status - Point 3 is at pixel x = 115.90, y = 19.72

// davidnie@davidnie-Inspiron-5488:~/4231/ScrewDrivingBot1$ ros2 service call /arm_srv interfaces/srv/ArmCmd "{mode: 'hole', point: {x: 0.35, y: 0.52, z: 0.3}}"

// davidnie@davidnie-Inspiron-5488:~/4231/ScrewDrivingBot1$ ros2 service call /arm_srv interfaces/srv/ArmCmd "{mode: 'hole', point: {x: 0.32, y: 0.55, z: 0.1}}"
// requester: making request: interfaces.srv.ArmCmd_Request(mode='hole', point=geometry_msgs.msg.Point(x=0.32, y=0.55, z=0.1))

// response:
// interfaces.srv.ArmCmd_Response(success=False)

// ros2 service call /arm_srv interfaces/srv/ArmCmd "{mode: 'home', point: {x: 0.0, y: 0.0, z: 0.0}}"

// Transform: x= 0.445403, y=0.513971, z=0.022418