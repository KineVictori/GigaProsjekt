#include <memory>
#include <iostream>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

class JointTargetSubscriberNode : public rclcpp::Node {
public:
  JointTargetSubscriberNode()
    : Node("joint_position_mover", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
      this->set_parameter(rclcpp::Parameter("use_sim_time", false));

      RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface");
      move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");

      auto initial_pose = move_group_interface_->getCurrentPose();

      RCLCPP_INFO(this->get_logger(), "Initial position: x=%.3f, y=%.3f, z=%.3f", 
                  initial_pose.pose.position.x, 
                  initial_pose.pose.position.y, 
                  initial_pose.pose.position.z);

      RCLCPP_INFO(this->get_logger(), "Initial orientation (quaternion): x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                  initial_pose.pose.orientation.x,
                  initial_pose.pose.orientation.y,
                  initial_pose.pose.orientation.z,
                  initial_pose.pose.orientation.w);

      subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_robot_pose", 10,
        std::bind(&JointTargetSubscriberNode::joint_callback, this, std::placeholders::_1)
      );
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;

    void joint_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      const std::vector<double>& poseTarget = msg->data;

      if (poseTarget.size() != 7) {
        RCLCPP_WARN(this->get_logger(), "Received %zu pose values, but expected 7!", poseTarget.size());
        return;
      }

      auto pose = move_group_interface_->getCurrentPose();

      pose.pose.position.x = poseTarget.at(0);
      pose.pose.position.y = poseTarget.at(1);
      pose.pose.position.z = poseTarget.at(2);

      pose.pose.orientation.x = poseTarget.at(3);
      pose.pose.orientation.y = poseTarget.at(4);
      pose.pose.orientation.z = poseTarget.at(5);
      pose.pose.orientation.w = poseTarget.at(6);

      move_group_interface_->setPoseTarget(pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = static_cast<bool>(move_group_interface_->plan(plan));

      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
        move_group_interface_->execute(plan);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Planning failed.");
      }
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<JointTargetSubscriberNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
