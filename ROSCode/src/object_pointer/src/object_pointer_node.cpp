#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <rclcpp/rclcpp.hpp>

class ObjectPointerNode : public rclcpp::Node {
public:
  ObjectPointerNode()
    : Node("object_pointer_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
      auto target_listener = [this](const geometry_msgs::msg::Point &point) {
        std::vector<double> data(7);

        data.at(0) = point.x;
        data.at(1) = point.y;
        data.at(2) = point.z - 0.1;
        data.at(3) = 0;
        data.at(4) = 0;
        data.at(5) = 0;
        data.at(6) = 1;

        std_msgs::msg::Float64MultiArray pose;
        pose.set__data(data);
        pose_publisher_->publish(pose);
      };

      target_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
          "/object_position", 10, target_listener);

      pose_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/target_robot_pose", 10);
    }

private:

private:
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pose_publisher_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ObjectPointerNode>());
	rclcpp::shutdown();
	return 0;
}
