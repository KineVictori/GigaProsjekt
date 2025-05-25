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

  // Get current joint values
  
  std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();

  std::cout << "Num jointes: " << joint_group_positions.size() << "\n";

  for (int i = 0; i < joint_group_positions.size(); i++) {
    joint_group_positions[i] += 0.3;
  }

  // Modify one joint slightly (e.g., add 0.1 radians to wrist_1_joint)
  //joint_group_positions[3] += 0.1;  // wrist_1_joint index

  // Set the new target
  move_group_interface.setJointValueTarget(joint_group_positions);

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
