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

// Ground Truth
// ros2 service call /arm_srv interfaces/srv/ArmCmd "{mode: 'hole', point: {x: 0.38, y: 0.3, z: 0.3}}"

// Test after tuning

//  Transform: x= 0.449440, y=0.513022, z=0.019677
// x= 0.447579, y=0.513695, z=0.020506
// Transform: x= 0.235417, y=0.579198, z=0.020389
// x= 0.233571, y=0.580123, z=0.020485
// x= 0.230881, y=0.578295, z=0.022316
// x= 0.448046, y=0.483478, z=0.019652

//  Transform: x= 0.248034, y=0.392805, z=0.020505
// Transform: x= 0.248632, y=0.483991, z=0.020801
// Transform: x= 0.450061, y=0.393995, z=0.019845

// Transform: x= 0.322784, y=0.279542, z=0.004010

// Transform: x= 0.323748, y=0.282129, z=0.000611
// x= 0.317764, y=0.274401, z=0.002971

// Transform: x= 0.602476, y=0.414721, z=0.020102

// with pos x
// Transform: x= 0.603776, y=0.454314, z=0.020353

//ground truth
// x: 0.5024, y: 0.2647, z: 0.4

// with neg x
// x= 0.601468, y=0.416548, z=0.021305

// test2

// Transform: x= 0.373479, y=0.526225, z=0.020193

//ground truth
// os2 service call /arm_srv interfaces/srv/ArmCmd "{mode: 'hole', point: {x: 0.27, y: 0.3762, z: 0.2}}"

// x - 0.1
// y - 0.15


// brain test

// x= 0.270318, y=0.376349, z=0.300000

// x: 0.315736, y: 0.462781, z: 0.3

// x=0.3535, y=0.5427, z=0.02

// realPose.position.x = realPose.position.x - 0.1;
// realPose.position.y = realPose.position.y - 0.15;

// realx = 0.415736
// realy = 0.512781

// to-screwhole
// -0.06
// +0.03