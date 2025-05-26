#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <rclcpp/rclcpp.hpp>

class ObjectPointerNode : public rclcpp::Node {
public:
  ObjectPointerNode()
    : Node("object_pointer_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
      auto target_listener = [this](const geometry_msgs::msg::Point &point) {
        geometry_msgs::msg::Pose msg;
        msg.position.x = point.x;
        msg.position.y = point.y;
        msg.position.z = point.z + 0.1;
        msg.orientation.x = 0;
        msg.orientation.y = 0;
        msg.orientation.z = 0;
        msg.orientation.w = 1;

        pose_publisher_->publish(msg);
      };

      target_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
          "/object_position", 10, target_listener);

      pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/target_robot_pose", 10);
    }

private:

private:
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ObjectPointerNode>());
	rclcpp::shutdown();
	return 0;
}
