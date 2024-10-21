#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/vision_cmd.hpp"
#include <string>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <tf2_ros/transform_listener.h>

class Brain : public rclcpp::Node
{
public:
	Brain() : Node("brain")
	{
	}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Brain>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}