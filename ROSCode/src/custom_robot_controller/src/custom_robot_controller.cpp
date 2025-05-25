#include <memory>
#include <iostream>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "small_joint_move",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  node->set_parameter(rclcpp::Parameter("use_sim_time", false));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("SmallJointMove");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group_interface(node, "ur_manipulator");

  
  std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();

  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();
  // Z: up/down, positive direction up
  // X: forward/back, if sitting at table, with robot in front of you, positive direction away from self
  // Y: left/right, same setting as X, positive is right.

  current_pose.pose.position.z += 0.05;
  current_pose.pose.position.x -= 0.05;
  current_pose.pose.position.y -= 0.05;
  move_group_interface.setPoseTarget(current_pose);

  // Plan and execute
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface.plan(plan));

  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful, executing...");
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}