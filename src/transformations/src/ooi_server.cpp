#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>

// set up dynamic transformation between camera_link
class OOIServer : public rclcpp::Node
{
	public:
		OOIServer() : Node("camera_dy_tf_broadcaster")
		{
			timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&OOIServer::publishDynamicTransform, this));
			tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
		}

	private:
		void publishDynamicTransform()
		{
			geometry_msgs::msg::TransformStamped transform_msg;

			std_msgs::msg::Header header;
			header.frame_id = "4231_camera_socket";
			header.stamp = now();

			geometry_msgs::msg::Vector3 vec;
			vec.x = 0;
			vec.y = 0;
			vec.z = 0;

			geometry_msgs::msg::Quaternion quat;
			quat.x = 0;
			quat.y = 0;
			quat.z = 0;
			quat.w = 1;

			geometry_msgs::msg::Transform trans;
			trans.translation = vec;
			trans.rotation = quat;

			transform_msg.header = header;
			transform_msg.child_frame_id = "camera_link";
			transform_msg.transform = trans;
			tf_broadcaster_->sendTransform(transform_msg);
		}

		rclcpp::TimerBase::SharedPtr timer_;
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OOIServer>());
	rclcpp::shutdown();
	return 0;
}