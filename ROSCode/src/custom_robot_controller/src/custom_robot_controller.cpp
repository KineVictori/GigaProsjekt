#include <memory>
#include <iostream>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

class JointTargetSubscriberNode : public rclcpp::Node {
public:
  JointTargetSubscriberNode()
    : Node("joint_position_mover", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
      this->set_parameter(rclcpp::Parameter("use_sim_time", false));
      

      subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/target_robot_pose", 10,
        std::bind(&JointTargetSubscriberNode::joint_callback, this, std::placeholders::_1)
      );
    }

  void init_move_group() {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface");

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
  }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;

    void joint_callback(const geometry_msgs::msg::Pose &pose) {
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

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<JointTargetSubscriberNode>();
  
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   auto spinner = std::thread([&executor]() { executor.spin(); });

//   node->init_move_group();

//   rclcpp::shutdown();
//   spinner.join();
//   return 0;
// }


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTargetSubscriberNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  node->init_move_group();

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}


