#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include "interfaces/srv/real_coor_cmd.hpp"

// set up dynamic transformation between camera_link
class OOIServer : public rclcpp::Node
{
	public:
		OOIServer() : Node("camera_dy_tf_broadcaster")
		{
			tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

			initTrans();

			ooiService = create_service<interfaces::srv::RealCoorCmd>(
				"ooiService",
				std::bind(&OOIServer::processOOIService, this, std::placeholders::_1, std::placeholders::_2));

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

		void processOOIService(const std::shared_ptr<interfaces::srv::RealCoorCmd::Request> request,
				 std::shared_ptr<interfaces::srv::RealCoorCmd::Response> response) {
			geometry_msgs::msg::Pose inputPose = request->img_pose;

			double x = inputPose.position.x;
			double y = inputPose.position.y;
			double z = inputPose.position.z;

			processAndSendTransform(x, y, z);

			response->success = true;
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

			tf_broadcaster_->sendTransform(transform_stamped);
		}

		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

		rclcpp::Service<interfaces::srv::RealCoorCmd>::SharedPtr ooiService;
		geometry_msgs::msg::TransformStamped transform_stamped;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OOIServer>());
	rclcpp::shutdown();
	return 0;
}